#include <ros_interface.h>

namespace XROS_TEMPLATE
{
/*------------------------------------------------------------------------------------------------------------------
 * name: BaseInit
-----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::BaseInit(int argc, char* argv[], std::string node_name,double period, void (*handle)())
{
    rclcpp::init(argc, argv);
    m_node_ptr = std::make_shared<rclcpp::Node>(node_name);

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
    // timer
    m_timer.reset();

    // pub
    pub_out_str.reset();

    // sub
    sub_in_str.reset();

    // node
    m_node_ptr.reset();

    // shutdown
    Shutdown();
}
/*------------------------------------------------------------------------------------------------------------------
 * name: NodeCheck
-----------------------------------------------------------------------------------------------------------------*/
bool ROSInterface::NodeCheck(const std::string& node_name)
{
    std::vector<std::string> nodes = m_node_ptr->get_node_names();
    for (size_t i = 0; i < nodes.size(); i++)
    {
        // ROSLog(LogLevel::kInfo," <%s> ",nodes[i].data());
        if (nodes[i] == "/" + node_name) return true;
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
        RCLCPP_FATAL(m_node_ptr->get_logger(), "### Wrong Log Message ###");
        return;
    }

    switch (level)
    {
        case LogLevel::kDebug:
            RCLCPP_DEBUG(m_node_ptr->get_logger(), "%s", buffer);
            break;
        case LogLevel::kInfo:
            RCLCPP_INFO(m_node_ptr->get_logger(), "%s", buffer);
            break;
        case LogLevel::kWarn:
            RCLCPP_WARN(m_node_ptr->get_logger(), "%s", buffer);
            break;
        case LogLevel::kError:
            RCLCPP_ERROR(m_node_ptr->get_logger(), "%s", buffer);
            break;
        case LogLevel::kFatal:
            RCLCPP_FATAL(m_node_ptr->get_logger(), "%s", buffer);
            break;
        default:
            RCLCPP_FATAL(m_node_ptr->get_logger(), "### Wrong Log Level ###");
            RCLCPP_FATAL(m_node_ptr->get_logger(), "%s", buffer);
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
    m_timer = m_node_ptr->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(period)), std::bind(&ROSInterface::TimerCallback, this));;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: ParameterInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::ParameterInit()
{
    m_node_ptr->declare_parameter<std::string>("out_str", "");
    m_node_ptr->get_parameter("out_str", m_out_str);
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
    pub_out_str =  m_node_ptr->create_publisher<std_msgs::msg::String>("/out", 50);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: SubscriptionInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::SubscriptionInit()
{ 
    sub_in_str = m_node_ptr->create_subscription<std_msgs::msg::String>("/in", 50, std::bind(&ROSInterface::InStrCallback, this, std::placeholders::_1));
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PubOutStr
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PubOutStr(const std::string& data)
{
    std_msgs::msg::String buff;
    buff.data.append(data);
    pub_out_str->publish(buff);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: InStrCallback
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::InStrCallback(const std_msgs::msg::String::SharedPtr data)
{
  if (!m_f_in_str) {
    m_in_str = data->data;
    m_f_in_str = true;
  }
}
}//namespace HEXROS
