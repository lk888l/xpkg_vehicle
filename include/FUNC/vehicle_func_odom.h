#ifndef VEHICLE_FUNC_ODOM_H
#define VEHICLE_FUNC_ODOM_H

#include <lib_file_ini.h>
#include <ros_interface.h>
#include <math.h>

#define MODE_ANGLE_VEL 0
#define MODE_ANGLE 1
/////////////////////////////////////////////////
struct  DataOdom{
  double odom_r = 0.0;
  double odom_l = 0.0;
  double odom_r_s = 0.0;
  double odom_l_s = 0.0;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vthx = 0.0;    //steering angle of akm
  double vthz = 0.0;
};
//////////////////////////////////////////////////
class VehicleFuncOdom
{
public:
    explicit VehicleFuncOdom(void);
    static VehicleFuncOdom& GetVehicleFuncOdom();
    void BasiInit(unsigned char type);
    void ShowLoc(void);
    void ResetLoc(void);
    void PubDevOdom(void);
    void CalcWithSpeed(double v_x,double v_y,double v_r,double time_now,int serial_num,unsigned char rotate_mode);
    void CalcWithOdom(double v_r,double odom_l,double odom_r,double odom_l_s,double odom_r_s,double time_now,int serial_num,unsigned char rotate_mode);

public:
    DataOdom m_data_odom;
    bool m_set_zero;
    double m_track_width;
    double m_wheel_base;
    std::string m_model;
};

#endif // VEHICLE_FUNC_ODOM_H
