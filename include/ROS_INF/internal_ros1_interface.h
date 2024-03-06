#ifndef ROS_INF_INTERNAL_ROS1_INTERFACE_H_
#define ROS_INF_INTERNAL_ROS1_INTERFACE_H_

#include <ros/ros.h>
#include <stdarg.h>

#include <memory>
#include <string>
#include <vector>

// add message include below ///////////////////////////////
// #include <xpkg_msgs/XmsgCommData.h>
#include <std_msgs/String.h>

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
  inline void Work() { ros::spin(); }
  inline void WorkOnce() { ros::spinOnce(); }
  inline void Shutdown() { ros::shutdown(); }
  inline bool Ok() { return ros::ok(); }
  inline ros::Time GetTime() { return ros::Time::now(); }
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
  inline void TimerCallback(const ros::TimerEvent&) { m_timer_handle(); }

 private:
  ros::NodeHandle* m_node_ptr;
  ros::NodeHandle* m_node_local_ptr;
  ros::Timer m_timer;
  void (*m_timer_handle)();

  /*********************************************************
                        Custom zone
  *********************************************************/
 public:
  // add pub function below ////////////////////////////////
  void PubOutStr(const std::string& data);

  // add sub function below ////////////////////////////////
  inline bool GetInStrFlag() { return m_f_in_str; }
  inline void ResetInStrFlag() { m_f_in_str = false; }
  inline std::string GetInStr() { return m_in_str; }

  // add sub callback below ////////////////////////////////
  void InStrCallback(const std_msgs::StringPtr& data);

 public:
  // add param Variable below //////////////////////////////
  std::string m_out_str;

 private:
  // add sub flag below ////////////////////////////////////
  bool m_f_in_str;

  // add pub Variable below ////////////////////////////////
  ros::Publisher pub_out_str;

  // add sub Variable below ////////////////////////////////
  ros::Subscriber sub_in_str;

  // add nomal Variable below //////////////////////////////
  std::string m_in_str;
};

}  // namespace XROS_TEMPLATE
#endif  // ROS_INF_INTERNAL_ROS1_INTERFACE_H_
