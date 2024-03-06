#ifndef VEHICLE_FUNC_H
#define VEHICLE_FUNC_H

#include <unistd.h>
#include <string>
#include <vector>
#include <lib_file_ini.h>
#include <ArduinoJson.h>
#include <ros_interface.h>

using namespace std;
using namespace XROS_VEHICLE;
////////////////////////////info of vehicle//////////////////////////////////////
#define MODE_STANDBY 0
#define MODE_REMOTE 1
#define MODE_CAN 2
#define MODE_FREE 3

#define BEEP_OFF 0
#define BEEP_ON 1

struct VehicleInfo
{
    //vehicle info
    unsigned char s_type = 0;
    unsigned char s_num = 0;
    //hardware and software version
    std::string s_ver_hard = "--";
    std::string s_ver_soft = "--";
    //system status
    bool s_en_state = false;   //enable state
    bool s_work_state = true;    //0x00 fine,0x01 error
    bool s_beep_state = false;
    bool s_brake_state = false;
    bool s_func_state = false;
    unsigned char s_work_mode = 0xFF;     //wokring mode:0x00 standby,0x01 remote,0x02 CAN,0x03 free
    unsigned char s_rotate_mode = 0x00;   //angle_vel:0x00 angel:0x01
    float s_bat_vol = 0.0;     // V
    unsigned char s_err_motor = 0;
    unsigned char s_err_driver_offline = 0;
    unsigned char s_err_driver = 0;
    unsigned char s_err_power = 0;
    bool s_err_bat_down = true; //<22V
    bool s_err_bat_low = true;  //<23V
    bool s_err_ctrl_lost = true;
    bool s_err_bump = true;
    bool s_err_stop = true;
    //motion status
    double s_speed_x = 0.0;   //m/s
    double s_speed_y = 0.0;   //m/s
    double s_speed_r = 0.0;   //rad/s
    //contraller status
    unsigned char s_ctrl_swa = 0x00;     //0x01 up,0x02 mid,0x03 down
    unsigned char s_ctrl_swb = 0x00;     //0x01 up,0x02 mid,0x03 down
    unsigned char s_ctrl_swc = 0x00;     //0x01 up,0x02 mid,0x03 down
    unsigned char s_ctrl_swd = 0x00;     //0x01 up,0x02 mid,0x03 down
    unsigned char s_ctrl_left_h = 0x00;  //-100~100
    unsigned char s_ctrl_left_v = 0x00;  //-100~100
    unsigned char s_ctrl_right_h = 0x00; //-100~100
    unsigned char s_ctrl_right_v = 0x00; //-100~100
    unsigned char s_ctrl_left_vra = 0x00; //-100~100
    unsigned char s_ctrl_right_vra = 0x00; //-100~100
    //odometer
    double s_odo_left = 0.0;   //m
    double s_odo_right = 0.0;  //m
    double s_odo_left_s = 0.0;  //m
    double s_odo_right_s = 0.0;  //m
    //bumper
    bool s_bump[8] = {0};
    //sonic
    unsigned char s_sonic[6] = {0};
    //driver status
    short s_driver_speed[4] = {0};    //PRM
    float s_driver_current[4] = {0};  //*0.1A
    int s_driver_pulse[4] = {0};
    float s_driver_volt[4] = {0};     //*0.1V
    char s_driver_heat[4] = {0};     //1C
    char s_motor_heat[4] = {0};     //1C
    bool s_driver_err_volt_low[4] = {1,1,1,1};
    bool s_driver_err_heat_over[4] = {1,1,1,1};
    bool s_motor_err_current_over[4] = {1,1,1,1};
    bool s_motor_err_heat_over[4] = {1,1,1,1};
    double s_time = 0.0;
    int s_serial = 0;
};
//////////////////////////////////////////////////////////////////
class VehicleFunc
{
public:
    explicit VehicleFunc(void);
    static VehicleFunc& GetVehicleFunc();
    inline VehicleInfo GetVehicleInfo(void){ return m_dev_info; }
    inline int IsOnline(void){ return f_online; }
    inline int IsEnable(void){ return f_enable; }
    inline int IsModeOn(void){ return f_mode; }
    inline int IsReady(void){ return f_ready; }
    void BaseInit(void);
    void ComDataOut();
    void DevInfoOut();
    void VelDataIn();
    void ComDataIn();
    void CtrlDataIn();
    void OnlineCheck();
    void EnableCheck();
    void ModeCheck();

    void DevReset();
    void DevClear();
    void DevVersion();
    void DevMode(unsigned char mode,bool beep,bool brake,bool func);
    void DevMove(double speed_x,double speed_y,double rotate);
    void DevSetEnable(bool en);
    void DevCtrlInfo(unsigned char state);
    void DevSensorInfo(unsigned char state);
    void DevDrvMotion(unsigned char state);
    void DevDrvState(unsigned char state);
    void DevZero();

public:
    VelData m_data_vel;
    std::string m_data_ctrl;

private:
    VehicleInfo m_dev_info;
    bool f_online;
    bool f_enable;
    bool f_mode;
    bool f_new_vel;
    bool f_ready;
    int m_online_count;
};

#endif // VEHICLE_FUNC_H
