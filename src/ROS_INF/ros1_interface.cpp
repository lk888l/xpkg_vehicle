#include <ros_interface.h>

namespace XROS_TEMPLATE
{
/*------------------------------------------------------------------------------------------------------------------
 * name: BaseInit
-----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::BaseInit(int argc, char* argv[], std::string node_name,double period, void (*handle)())
{
    ros::init(argc, argv, node_name);
    static ros::NodeHandle nh;
    static ros::NodeHandle nh_local("~");
    m_node_ptr = &nh;
    m_node_local_ptr = &nh_local;

    VariableInit();
    ParameterInit();
    PublisherInit();
    SubscriptionInit();
    TimerInit(period, handle);

    ROSLog(LogLevel::kInfo, "\033[1;32m %s: ### ROS interface init finish ### \033[0m", node_name.data());
}
/*------------------------------------------------------------------------------------------------------------------
 * name: BaseDeinit
-----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::BaseDeinit()
{
    ros::shutdown();
}
/*------------------------------------------------------------------------------------------------------------------
 * name: NodeCheck
-----------------------------------------------------------------------------------------------------------------*/
bool ROSInterface::NodeCheck(const std::string& node_name)
{
    std::vector<std::string> nodes;
    ros::master::getNodes(nodes);
    for (size_t i = 0; i < nodes.size(); i++)
    {
        // ROSLog(LogLevel::kInfo," <%s> ",nodes[i].data());
        if (nodes[i] == "/"+node_name) return true;
    }

    return false;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: GetInterface
 -----------------------------------------------------------------------------------------------------------------*/
ROSInterface& ROSInterface::GetInterface()
{
    static ROSInterface ros_interface;
    return ros_interface;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: ROSLog
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::ROSLog(LogLevel level, const char* format, ...)
{
    char* buffer;

    va_list args;
    va_start(args, format);
    int32_t len = vasprintf(&buffer, format, args);
    va_end(args);

    if (len < 0)
    {
        ROS_FATAL("### Wrong Log Message ###");
        return;
    }

    switch (level)
    {
        case LogLevel::kDebug:
            ROS_DEBUG("%s", buffer);
            break;
        case LogLevel::kInfo:
            ROS_INFO("%s", buffer);
            break;
        case LogLevel::kWarn:
            ROS_WARN("%s", buffer);
            break;
        case LogLevel::kError:
            ROS_ERROR("%s", buffer);
            break;
        case LogLevel::kFatal:
            ROS_FATAL("%s", buffer);
            break;
        default:
            ROS_FATAL("### Wrong Log Level ###");
            ROS_FATAL("%s", buffer);
            break;
    }
    free(buffer);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: TimerInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::TimerInit(double period, void (*handle)())
{
    m_timer_handle = handle;
    m_timer = m_node_ptr->createTimer(ros::Duration(period * 0.001), &ROSInterface::TimerCallback, this);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: ParameterInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::ParameterInit()
{
    m_node_local_ptr->param<std::string>("out_str", m_out_str, "");
}
/*------------------------------------------------------------------------------------------------------------------
 * name: VariableInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::VariableInit()
{
    m_f_in_str = false;
    m_in_str = "";
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PublisherInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PublisherInit()
{
    pub_out_str = m_node_local_ptr->advertise<std_msgs::String>("/out", 50);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: SubscriptionInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::SubscriptionInit()
{ 
    sub_in_str = m_node_local_ptr->subscribe("/in", 50, &ROSInterface::InStrCallback, this);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PubOutStr
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PubOutStr(const std::string& data)
{
    std_msgs::String buff;
    buff.data.append(data);
    pub_out_str.publish(buff);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: InStrCallback
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::InStrCallback(const std_msgs::StringPtr& data)
{
  if (!m_f_in_str) {
    m_in_str = data->data;
    m_f_in_str = true;
  }
}
}//namespace HEXROS
