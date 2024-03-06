#ifndef ROS_INF_INTERNAL_ROS2_INTERFACE_H_
#define ROS_INF_INTERNAL_ROS2_INTERFACE_H_

#include <stdarg.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

// add message include below ///////////////////////////////
//  #include <xpkg_msgs/msg/xmsg_comm_data.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std;
namespace XROS_TEMPLATE {
////////////////////////////////////////////////////////////
enum class LogLevel { kDebug = 0, kInfo, kWarn, kError, kFatal };
////////////////////////////////////////////////////////////
class ROSInterface {
  /*********************************************************
                    Base function zone
  *********************************************************/
 public:
  static ROSInterface& GetInterface();
  void BaseInit(int argc, char* argv[], std::string node_name, double period,
                void (*handle)());
  void BaseDeinit();
  bool NodeCheck(const std::string& node_name);
  inline void Work() { rclcpp::spin(m_node_ptr); }
  inline void Shutdown() { rclcpp::shutdown(); }
  inline bool Ok() { return rclcpp::ok(); }
  inline rclcpp::Time GetTime() { return m_node_ptr->now(); }
  void ROSLog(LogLevel, const char*, ...);

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
  // add pub function below ////////////////////////////////
  void PubOutStr(const std::string& data);

  // add sub function below/////////////////////////////////
  inline bool GetInStrFlag() { return m_f_in_str; }
  inline void ResetInStrFlag() { m_f_in_str = false; }
  inline std::string GetInStr() { return m_in_str; }

  // add sub callback below ////////////////////////////////
  void InStrCallback(const std_msgs::msg::String::SharedPtr data);

 public:
  // add param Variable below //////////////////////////////
  std::string m_out_str;

 private:
  // add sub flag below ////////////////////////////////////
  bool m_f_in_str;

  // add pub Variable below ////////////////////////////////
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_out_str;

  // add sub Variable below ////////////////////////////////
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_in_str;

  // add nomal Variable below //////////////////////////////
  std::string m_in_str;
};

}  // namespace XROS_TEMPLATE
#endif  // ROS_INF_INTERNAL_ROS2_INTERFACE_H_
