#!/usr/bin/env python3
"""Bounded lifted-wheel acceptance; explicitly use --motion for short wheel runs.

Owns an isolated ROS master and nodes. Every exit attempts zero + disable, then
driver shutdown repeats zero + disable. Never runs the body-yaw calibration.
"""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import threading
import time


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--interface', default='can0')
    p.add_argument('--port', type=int, default=11331)
    p.add_argument('--motion', action='store_true')
    p.add_argument('--use-imu', choices=('true', 'false'), default='true')
    p.add_argument('--output', type=Path, required=True)
    args=p.parse_args()
    with socket.socket() as probe:
        if probe.connect_ex(('127.0.0.1',args.port))==0:
            p.error('Test ROS master port is occupied')
    os.environ['ROS_MASTER_URI']='http://127.0.0.1:'+str(args.port)
    os.environ['ROS_IP']='127.0.0.1'
    os.environ.pop('ROS_HOSTNAME',None)
    import rosgraph
    import rospy
    from roslib.packages import find_node
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from diagnostic_msgs.msg import DiagnosticArray
    from vehicle_interfaces.msg import State, DriverMotion, DriverStatus, ImuStatus
    from vehicle_interfaces.srv import Command, CommandRequest

    args.output.parent.mkdir(parents=True,exist_ok=True)
    report={'utc':datetime.now(timezone.utc).isoformat(),'interface':args.interface,
            'motion_requested':args.motion,'use_imu':args.use_imu == 'true',
            'commands':[],'phases':[],'completed':False}
    states=[];velocities=[];wheels=[];electrical=[];imu_status=[];imu_data=[];odometry=[]
    diagnostics=[]
    processes=[];subs=[];proxy=None;publisher=None
    bus=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
    bus.bind((args.interface,));bus.settimeout(.05)
    frames=[];counts=Counter();running=threading.Event();running.set()
    def capture():
        while running.is_set():
            try:
                f,_,flags,_=bus.recvmsg(72)
                identifier,length=struct.unpack_from('=IB',f)
                counts[hex(identifier)]+=1
                frames.append((time.monotonic(),identifier,f[8:8+length].hex(),bool(flags&socket.MSG_DONTROUTE)))
            except socket.timeout:
                pass
    monitor=threading.Thread(target=capture,daemon=True);monitor.start()

    def wait(predicate, timeout, description):
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            if predicate(): return
            if processes and any(x.poll() is not None for x in processes):
                raise RuntimeError('Owned ROS process exited: '+description)
            time.sleep(.02)
        raise RuntimeError('Timed out: '+description)

    def call(code, **fields):
        r=proxy(CommandRequest(command=code,**fields))
        result={k:getattr(r,k) for k in r.__slots__}
        result['command']=code;result['request']=fields
        report['commands'].append(result)
        print(json.dumps(result,ensure_ascii=False),flush=True)
        if not r.sent: raise RuntimeError(r.message)
        return r

    def publish(vx=0.0,wz=0.0):
        m=TwistStamped();m.header.stamp=rospy.Time.now()
        m.twist.linear.x=vx;m.twist.angular.z=wz;publisher.publish(m)

    def ready():
        if not states:return False
        _,s=states[-1]
        return s.online and s.enabled and s.status_received and s.mode==2 and s.work_state==0 and \
            (rospy.Time.now()-s.heartbeat_stamp).to_sec()<1.0 and \
            (rospy.Time.now()-s.status_stamp).to_sec()<.5

    def stopped():
        latest={}
        for t,m in wheels:
            if time.monotonic()-t<.6:latest[m.channel]=m.rpm
        return len(latest)==4 and all(abs(x)<=3 for x in latest.values())

    def phase(name,vx,wz,duration,watchdog=False):
        start=time.monotonic();last_publish=0
        print('Phase '+name,flush=True)
        while time.monotonic()-start<duration:
            if not ready():raise RuntimeError('Chassis state became unsafe in '+name)
            publish(vx,wz);last_publish=time.monotonic();time.sleep(.05)
        if not watchdog:publish()
        # Stop must be confirmed by actual four-channel feedback.
        wait(stopped,2.5,'four wheel RPMs settle after '+name)
        end=time.monotonic()
        wheel_samples=[(t,m) for t,m in wheels if start<=t<=end]
        peaks={str(c):max((abs(m.rpm) for t,m in wheel_samples if m.channel==c),default=0) for c in range(4)}
        commands=[(t,struct.unpack('<hhh',bytes.fromhex(data))) for t,i,data,local in frames
                  if i==0x81e10112 and local and start<=t<=end and len(data)==12]
        zero=[t for t,value in commands if t>=last_publish and value==(0,0,0)]
        measured=[(m.twist.linear.x,m.twist.angular.z) for t,m in velocities if start<=t<=end]
        item={'name':name,'vx':vx,'wz':wz,'duration':duration,'wheel_peak_abs_rpm':peaks,
              'wheel_stopped':stopped(),'velocity_samples':measured,
              'stop_command_delay_s':zero[0]-last_publish if zero else None}
        report['phases'].append(item)
        if max(peaks.values())<=3:raise RuntimeError('No measured wheel response in '+name)
        if not zero:raise RuntimeError('No zero-speed CAN frame after '+name)
        if watchdog and not .20<=item['stop_command_delay_s']<=.45:
            raise RuntimeError('Unexpected watchdog timing')
        time.sleep(.4)

    def stop_process(proc):
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.terminate();proc.wait(timeout=3)

    def abort(signum,frame):raise RuntimeError('Bench test interrupted or timed out')
    old_int=signal.signal(signal.SIGINT,abort)
    old_term=signal.signal(signal.SIGTERM,abort)
    signal.signal(signal.SIGALRM,abort);signal.alarm(70)
    log=args.output.with_suffix('.log').open('w')
    try:
        wait(lambda:counts['0x81e101b0']>=2,3,'real chassis heartbeat before launching control')
        processes.append(subprocess.Popen(['roscore','-p',str(args.port)],stdout=log,stderr=subprocess.STDOUT))
        wait(rosgraph.is_master_online,8,'ROS master')
        rospy.init_node('hardware_bench_observer',disable_signals=True)
        for cls,topic,dest in ((State,'/bench/vehicle/state',states),
                (TwistStamped,'/bench/vehicle/velocity',velocities),
                (DriverMotion,'/bench/vehicle/driver_motion',wheels),
                (DriverStatus,'/bench/vehicle/driver_status',electrical),
                (ImuStatus,'/bench/imu/status',imu_status),(Imu,'/bench/imu/data',imu_data),
                (DiagnosticArray,'/bench/diagnostics',diagnostics),
                (Odometry,'/bench/odom',odometry)):
            subs.append(rospy.Subscriber(topic,cls,lambda m,d=dest:d.append((time.monotonic(),m)),queue_size=200))
        publisher=rospy.Publisher('/bench/cmd_vel',TwistStamped,queue_size=1,tcp_nodelay=True)
        # Deliberately omit receive_only: this validates the node's control default.
        nodes=[('vehicle_node',['_auto_initialize:=false',
                   '_auto_imu_calibration:=false','_max_linear_speed:=0.10','_max_angular_speed:=0.20',
                   '_publish_tf:=false','_can_interface:='+args.interface,
                   '_use_imu_heading:='+args.use_imu])]
        if args.use_imu == 'true':
            nodes.append(('imu_node',['_imu_protocol:=classic','_imu_can_interface:='+args.interface]))
        for binary,params in nodes:
            processes.append(subprocess.Popen([find_node('vehicle_driver',binary)[0],'__ns:=/bench']+params,
                                               stdout=log,stderr=subprocess.STDOUT))
        rospy.wait_for_service('/bench/vehicle/command',timeout=6)
        proxy=rospy.ServiceProxy('/bench/vehicle/command',Command)
        wait(lambda:states and states[-1][1].online,3,'chassis online')
        wait(lambda:diagnostics,3,'driver diagnostics')
        effective={item.key:item.value for item in diagnostics[-1][1].status[0].values}
        report['receive_only']=effective.get('receive_only')
        if report['receive_only'] != '0':raise RuntimeError('Default control remains blocked')
        version=call(CommandRequest.QUERY_VERSION)
        if not version.acknowledged:raise RuntimeError('Version query not acknowledged')
        enabled=call(CommandRequest.ENABLE,enabled=True)
        if not(enabled.acknowledged and enabled.state_confirmed):raise RuntimeError('Enable not confirmed')
        mode=call(CommandRequest.SET_MODE,mode=2)
        if not mode.state_confirmed:raise RuntimeError('CAN mode not confirmed')
        call(CommandRequest.DRIVER_MOTION_FEEDBACK,enabled=True)
        call(CommandRequest.DRIVER_STATUS_FEEDBACK,enabled=True)
        wait(lambda:ready() and stopped() and publisher.get_num_connections()>0,4,'ready and four stopped wheels')
        report['battery_voltage']=states[-1][1].battery_voltage
        report['ready']=True
        if args.motion:
            for name,vx,wz in [('forward',.05,0),('reverse',-.05,0),('left',0,.15),('right',0,-.15)]:
                phase(name,vx,wz,1.2)
            phase('watchdog',.05,0,.8,watchdog=True)
        report['completed']=True
    except Exception as e:
        report['error']=str(e);print('Bench stopped: '+str(e),flush=True)
    finally:
        signal.alarm(0)
        if publisher is not None:
            try:
                for _ in range(4):publish();time.sleep(.05)
            except Exception as e:report['stop_error']=str(e)
        if proxy is not None:
            try:
                r=call(CommandRequest.ENABLE,enabled=False)
                report['disable_confirmed']=bool(r.state_confirmed)
                time.sleep(.6)
            except Exception as e:report['disable_error']=str(e)
        report['final_enabled']=states[-1][1].enabled if states else None
        report['final_wheels']={str(m.channel):m.rpm for t,m in wheels if time.monotonic()-t<1}
        report['imu_valid']=bool(imu_status and imu_status[-1][1].orientation_valid)
        report['final_driver_diagnostics']={item.key:item.value for item in diagnostics[-1][1].status[0].values} if diagnostics else {}
        report['telemetry_counts']={k:len(v) for k,v in [('state',states),('velocity',velocities),
                           ('driver_motion',wheels),('driver_status',electrical),('odometry',odometry)]}
        for proc in reversed(processes):stop_process(proc)
        for sub in subs:sub.unregister()
        if publisher is not None:publisher.unregister()
        time.sleep(.1);running.clear();monitor.join(timeout=1);bus.close();log.close()
        report['can_frame_counts']=dict(counts)
        args.output.with_suffix('.frames.json').write_text(json.dumps(frames))
        args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
        signal.signal(signal.SIGINT,old_int);signal.signal(signal.SIGTERM,old_term)
    print(json.dumps(report,indent=2,ensure_ascii=False))
    return 0 if report['completed'] and report.get('disable_confirmed') else 2


if __name__=='__main__':raise SystemExit(main())
