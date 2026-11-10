"""ROS1 test probes. Protocol emulators only accept verified kernel vcan links."""
from concurrent.futures import Future
from threading import Thread
import time

import rospy
from roslib.packages import find_node


def init_ros():
    if not rospy.core.is_initialized():
        rospy.init_node("vehicle_noetic_tests", disable_signals=True)


def executable(name):
    return find_node("vehicle_driver", name)[0]


class Client:
    def __init__(self, name, service_type):
        self.name = name
        self.proxy = rospy.ServiceProxy(name, service_type)

    def service_is_ready(self):
        try:
            rospy.wait_for_service(self.name, timeout=0.02)
            return True
        except rospy.ROSException:
            return False

    def call_async(self, payload):
        future = Future()
        def call():
            try:
                future.set_result(self.proxy(payload))
            except Exception as error:
                future.set_exception(error)
        Thread(target=call, daemon=True).start()
        return future


class Publisher:
    def __init__(self, message_type, name, depth):
        self.publisher = rospy.Publisher(name, message_type, queue_size=depth, tcp_nodelay=True)

    def publish(self, message):
        self.publisher.publish(message)

    def get_subscription_count(self):
        return self.publisher.get_num_connections()

    def unregister(self):
        self.publisher.unregister()


class RosProbe:
    def __init__(self, *args, **kwargs):
        init_ros()
        self.handles = []

    def create_subscription(self, message_type, name, callback, depth):
        sub = rospy.Subscriber(name, message_type, callback, queue_size=depth, tcp_nodelay=True)
        self.handles.append(sub)
        return sub

    def create_publisher(self, message_type, name, depth):
        pub = Publisher(message_type, name, depth)
        self.handles.append(pub)
        return pub

    def create_client(self, service_type, name):
        return Client(name, service_type)

    def create_service(self, service_type, name, callback):
        service = rospy.Service(name, service_type,
                                lambda request: callback(request, service_type._response_class()))
        self.handles.append(service)
        return service

    def destroy_subscription(self, subscription):
        subscription.unregister()
        self.handles.remove(subscription)

    def destroy_client(self, client):
        client.proxy.close()

    def destroy_node(self):
        for handle in self.handles:
            if isinstance(handle, rospy.Service):
                handle.shutdown()
            else:
                handle.unregister()
        self.handles.clear()
