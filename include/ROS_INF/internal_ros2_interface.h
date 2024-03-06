#pragma once
#ifndef INTERNAL_ROS2_INTERFACE_H
#define INTERNAL_ROS2_INTERFACE_H

#include <string>
#include <vector>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <stdarg.h>

//add message include below////////////////////////////////
#include <xpkg_msgs/msg/xmsg_comm_data.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_msgs/msg/string.hpp>


using namespace std;
namespace XROS_VEHICLE
{
////////////////////////////////////////////////////////////
enum class LogLevel
{
  kDebug = 0,
  kInfo,
  kWarn,
  kError,
  kFatal
};
struct XstdData
{
  unsigned char id_c;
  unsigned char id_t;
  unsigned char id_n;
  unsigned char id_f;
  unsigned char len;
  unsigned char data[8];
  double time;
};
struct VelData
{
  double line_x;
  double line_y;
  double line_z;
  double ang_x;
  double ang_y;
  double ang_z;
};
////////////////////////////////////////////////////////////
class ROSInterface
{
  /*********************************************************
                    Base function zone
  *********************************************************/
  public:
    static ROSInterface& GetInterface();
    void BaseInit(int argc, char* argv[], std::string node_name, double period, void (*handle)());
    void BaseDeinit();
    inline void Work() {rclcpp::spin(m_node_ptr);}
    inline void Shutdown() {rclcpp::shutdown();}
    inline bool Ok() {return rclcpp::ok();}
    inline rclcpp::Time GetTime() {return m_node_ptr->now();}
    void ROSLog(LogLevel, const char*, ...);
    bool NodeCheck(std::string node_name);
  
  private:
    ROSInterface() = default;
    virtual ~ROSInterface() = default;
    void VariableInit();
    void ParameterInit();
    void PublisherInit();
    void SubscriptionInit();
    void TimerInit(double period, void (*handle)());

  protected:
    inline void TimerCallback() { m_timer_handle(); }

  private:
    std::shared_ptr<rclcpp::Node> m_node_ptr;
    rclcpp::TimerBase::SharedPtr m_timer;
    void (*m_timer_handle)();

  /*********************************************************
                        Custom zone
  *********************************************************/
  public:
    //add pub function below////////////////////////////////
    void PubComXstd(const XstdData& data);
    void PubOdom(double lx,double ly,double th,double vx,double vy,double vthx,double vthz);
    void PubDevState(const std::string& data);

    //add sub function below////////////////////////////////
    inline bool GetComXstdFlag() { return m_f_com_xstd; }
    inline bool GetVelFlag() { return m_f_vel; }
    inline bool GetJsonCtrlFlag() { return m_f_json_ctrl; }
    inline void ResetComXstdFlag() { m_f_com_xstd = false; }
    inline void ResetVelFlag() { m_f_vel = false; }
    inline void ResetJsonCtrlFlag() { m_f_json_ctrl = false; }
    inline vector<XstdData> GetComXstdMsg() { return m_list_com_xstd; }
    inline vector<VelData> GetVelMsg() { return m_list_vel; }
    inline std::string GetJsonCtrlMsg() { return m_json_ctrl; }
    inline void ClearComXstdMsg() { m_list_com_xstd.clear(); }
    inline void ClearVelMsg() { m_list_vel.clear(); }
    inline void ClearJsonCtrlMsg() { m_json_ctrl.clear(); }

    //add sub callback below////////////////////////////////////
    void ComXstdCallback(const xpkg_msgs::msg::XmsgCommData::SharedPtr data);
    void VelCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);
    void JsonCtrlCallback(const std_msgs::msg::String::SharedPtr data);

  public:
    //add param Variable below///////////////////////////////
    std::string m_ini_path;
    bool m_pub_tf;
    bool m_show_loc;
    bool m_calc_speed;
    bool m_mode_can_lock;
    bool m_show_path;
    double m_rate_x;
    double m_rate_y;
    double m_rate_z;
    double m_rate_az; 

  private:
    //add sub flag below/////////////////////////////////////
    bool m_f_com_xstd;
    bool m_f_vel;
    bool m_f_json_ctrl;

    //add pub Variable below/////////////////////////////////
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    rclcpp::Publisher<xpkg_msgs::msg::XmsgCommData>::SharedPtr pub_com_xstd;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_device_state;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
    // tf
    std::unique_ptr<tf2_ros::TransformBroadcaster> pub_tf_;

    //add sub Variable below/////////////////////////////////
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_vel;
    rclcpp::Subscription<xpkg_msgs::msg::XmsgCommData>::SharedPtr sub_com_xstd;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_json_ctrl;

    //add nomal Variable below///////////////////////////////
    vector<XstdData> m_list_com_xstd;
    vector<VelData> m_list_vel;    
    std::string m_json_ctrl;
};

}//namespace XROS_VEHICLE
#endif // INTERNAL_ROS2_INTERFACE_H

