#include <ros_interface.h>

namespace XROS_VEHICLE
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

    ROSLog(LogLevel::kInfo,"\033[1;32m %s: ### ROS interface init finish ### \033[0m",node_name.data());
}
/*------------------------------------------------------------------------------------------------------------------
 * name: BaseDeinit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::BaseDeinit()
{
    // timer
    m_timer.reset();

    // pub
    pub_odom.reset();
    pub_com_xstd.reset();
    pub_device_state.reset();
    pub_path.reset();
    pub_tf_.reset();

    // sub
    sub_vel.reset();
    sub_com_xstd.reset();

    // node
    m_node_ptr.reset();

    // shutdown
    Shutdown();
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
    m_timer = m_node_ptr->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(period)), std::bind(&ROSInterface::TimerCallback, this));
}
/*------------------------------------------------------------------------------------------------------------------
 * name: NodeCheck
 -----------------------------------------------------------------------------------------------------------------*/
bool ROSInterface::NodeCheck(std::string node_name)
{
    std::vector<std::string> nodes = m_node_ptr->get_node_names();
    for(unsigned long i=0;i<nodes.size();i++)
    {
       // ROSLog(LogLevel::kInfo," <%s> ",nodes[i].data());
        if (nodes[i] == "/"+node_name)return true;
    }
    return false;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: ParameterInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::ParameterInit()
{
    m_node_ptr->declare_parameter<std::string>("ini_path", "");
    m_node_ptr->declare_parameter<bool>("show_loc", false);
    m_node_ptr->declare_parameter<bool>("show_path", false);
    m_node_ptr->declare_parameter<bool>("calc_speed", false);
    m_node_ptr->declare_parameter<bool>("mode_can_lock", false);
    m_node_ptr->declare_parameter<bool>("pub_tf", true);
    m_node_ptr->declare_parameter<double>("rate_x", 1.0);
    m_node_ptr->declare_parameter<double>("rate_y", 1.0);
    m_node_ptr->declare_parameter<double>("rate_z", 1.0);
    m_node_ptr->declare_parameter<double>("rate_az", 1.0);

    m_node_ptr->get_parameter("ini_path", m_ini_path);
    m_node_ptr->get_parameter("show_loc", m_show_loc);
    m_node_ptr->get_parameter("show_path", m_show_path);
    m_node_ptr->get_parameter("calc_speed", m_calc_speed);
    m_node_ptr->get_parameter("mode_can_lock", m_mode_can_lock);
    m_node_ptr->get_parameter("pub_tf", m_pub_tf);
    m_node_ptr->get_parameter("rate_x", m_rate_x);
    m_node_ptr->get_parameter("rate_y", m_rate_y);
    m_node_ptr->get_parameter("rate_z", m_rate_z);
    m_node_ptr->get_parameter("rate_az", m_rate_az);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: VariableInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::VariableInit()
{
    m_show_loc = false;
    m_show_path = false;
    m_calc_speed = false;
    m_mode_can_lock = true;
    m_rate_x = 1.0;
    m_rate_y = 1.0;
    m_rate_z = 1.0;
    m_rate_az = 1.0;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PublisherInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PublisherInit()
{
    pub_com_xstd = m_node_ptr->create_publisher<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/com_send_xstd", 50);
    pub_odom = m_node_ptr->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
    pub_device_state = m_node_ptr->create_publisher<std_msgs::msg::String>("/xtopic_vehicle/device_state_json", 50);
    pub_path =  m_node_ptr->create_publisher<nav_msgs::msg::Path>("/vehicle_path", 50);

    pub_tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*m_node_ptr);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: SubscriptionInit
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::SubscriptionInit()
{ 
    sub_com_xstd = m_node_ptr->create_subscription<xpkg_msgs::msg::XmsgCommData>("/xtopic_comm/com_recv_xstd_vehicle", 1000, std::bind(&ROSInterface::ComXstdCallback, this, std::placeholders::_1));
    sub_vel = m_node_ptr->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 50, std::bind(&ROSInterface::VelCallback, this, std::placeholders::_1));
    sub_json_ctrl = m_node_ptr->create_subscription<std_msgs::msg::String>("/xtopic_vehicle/ctrl_json", 50, std::bind(&ROSInterface::JsonCtrlCallback, this, std::placeholders::_1));
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PubComXstd
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PubComXstd(const XstdData& data)
{
    xpkg_msgs::msg::XmsgCommData data_com;
    data_com.len = data.len;
    data_com.id_c = data.id_c;
    data_com.id_t = data.id_t;
    data_com.id_n = data.id_n;
    data_com.id_f = data.id_f;
    memcpy(&data_com.data[0],&data.data[0],data.len);
    data_com.time = m_node_ptr->now();
    pub_com_xstd->publish(data_com);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PubOdom
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PubOdom(double lx,double ly,double th,double vx,double vy,double vthx,double vthz)
{
    rclcpp::Time current_time = m_node_ptr->now();
    //publish tf////////////////////////////
    tf2::Quaternion quat;
    quat.setRPY(0, 0, th);

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = current_time;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = lx;
    odom_tf.transform.translation.y = ly;
    odom_tf.transform.translation.z = 0.0;
    odom_tf.transform.rotation.x = quat.x();
    odom_tf.transform.rotation.y = quat.y();
    odom_tf.transform.rotation.z = quat.z();
    odom_tf.transform.rotation.w = quat.w();
    if(m_pub_tf)pub_tf_->sendTransform(odom_tf);

    //publish odom//////////////////////////
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = lx;
    odom.pose.pose.position.y = ly;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vthz;
    odom.twist.twist.angular.x = vthx;
    pub_odom->publish(odom);

    //publish path//////////////////////////
    if(m_show_path)
    {
        static nav_msgs::msg::Path path;
        geometry_msgs::msg::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = odom.pose.pose.position.x;
        this_pose_stamped.pose.position.y = odom.pose.pose.position.y;
        this_pose_stamped.pose.orientation = odom.pose.pose.orientation;
        this_pose_stamped.header.stamp = current_time;
        this_pose_stamped.header.frame_id = "odom";
        path.poses.push_back(this_pose_stamped);
        if(path.poses.size()>1000)path.poses.erase(path.poses.begin());

        path.header.stamp = m_node_ptr->now();
        path.header.frame_id = "odom";
        pub_path->publish(path);
    }
} 
/*------------------------------------------------------------------------------------------------------------------
 * name: PubDevList
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::PubDevState(const std::string& data)
{
    std_msgs::msg::String buff;
    buff.data.append(data);
    pub_device_state->publish(buff);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: sub callback
 -----------------------------------------------------------------------------------------------------------------*/
void ROSInterface::ComXstdCallback(const xpkg_msgs::msg::XmsgCommData::SharedPtr data)
{
    m_f_com_xstd = true;
    XstdData data_com;
    data_com.len = data->len;
    data_com.id_c = data->id_c;
    data_com.id_t = data->id_t;
    data_com.id_n = data->id_n;
    data_com.id_f = data->id_f;
    memcpy(&data_com.data[0], &data->data[0], data->len);
    data_com.time = m_node_ptr->now().seconds();
    m_list_com_xstd.push_back(data_com);
    if (m_list_com_xstd.size() > 500) m_list_com_xstd.clear();
}
///////////////////////////////////////////////////////////////////////////////////////////////
void ROSInterface::VelCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
{
    m_f_vel = true;
    VelData data_vel;
    data_vel.line_x = cmd_vel->linear.x;
    data_vel.line_y = cmd_vel->linear.y;
    data_vel.line_z = cmd_vel->linear.z;
    data_vel.ang_x = cmd_vel->angular.x;
    data_vel.ang_y = cmd_vel->angular.y;
    data_vel.ang_z = cmd_vel->angular.z;
    m_list_vel.push_back(data_vel);
    if (m_list_vel.size() > 500) m_list_vel.clear();
}
///////////////////////////////////////////////////////////////////////////////////////////////
void ROSInterface::JsonCtrlCallback(const std_msgs::msg::String::SharedPtr data)
{
    m_f_json_ctrl = true;
    m_json_ctrl.append(data->data);
}

}//namespace XROS_VEHICLE

