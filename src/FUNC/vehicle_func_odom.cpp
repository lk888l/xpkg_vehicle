#include <vehicle_func_odom.h>
using namespace XROS_VEHICLE;

VehicleFuncOdom::VehicleFuncOdom()
{
    m_set_zero =  false;
    m_track_width = 0;
    m_wheel_base = 0;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: GetVehicleFuncOdom
 -----------------------------------------------------------------------------------------------------------------*/
VehicleFuncOdom& VehicleFuncOdom::GetVehicleFuncOdom()
{
    static VehicleFuncOdom vehicle_func_odom;
    return vehicle_func_odom;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: BasiInit
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFuncOdom::BasiInit(unsigned char type)
{
    LibFileIni& lib_file_ini = LibFileIni::GetLibFileIni();
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    std::string calc_mode;
    ResetLoc();
    m_track_width = lib_file_ini.GetKeyDouble(lib_file_ini.ToHexStr(type).c_str(),"width");
    m_wheel_base = lib_file_ini.GetKeyDouble(lib_file_ini.ToHexStr(type).c_str(),"long");
    m_model = lib_file_ini.GetKeyValue(lib_file_ini.ToHexStr(type).c_str(),"model");
    if(ros_interface.m_calc_speed)calc_mode = "speed";
    else calc_mode = "odom";
    ros_interface.ROSLog(LogLevel::kInfo,"\033[1;34m xnode_vehicle: cal_mode = %s ||| model = %s \033[0m",
                         calc_mode.c_str(),m_model.c_str());
    ros_interface.ROSLog(LogLevel::kInfo,"\033[1;34m xnode_vehicle: track_width = %.3fm ||| wheel_base = %.3fm \033[0m",
                        m_track_width,m_wheel_base);
    ros_interface.ROSLog(LogLevel::kInfo,"\033[1;32m xnode_vehicle: ### Odom init finish ### \033[0m");
}
/*------------------------------------------------------------------------------------------------------------------
 * name: ShowLoc
 * detail: Print out location info
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFuncOdom::ShowLoc(void)
{
  ROSInterface& ros_interface = ROSInterface::GetInterface();
  if(ros_interface.m_show_loc == false)return;
  ros_interface.ROSLog(LogLevel::kInfo,"\033[1;34m Location info: x=%.3fm y=%.3fm th=%.1fdeg/%.3frad \033[0m",
                       m_data_odom.x,m_data_odom.y,m_data_odom.th/3.14*180,m_data_odom.th);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: ResetLoc
 * detail: Reset location
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFuncOdom::ResetLoc(void)
{
  m_data_odom.x = 0;
  m_data_odom.y = 0;
  m_data_odom.th = 0;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: PubDevOdom
 * detail: Pub odom ros msg
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFuncOdom::PubDevOdom()
{
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    ros_interface.PubOdom(m_data_odom.x,m_data_odom.y,m_data_odom.th,m_data_odom.vx,m_data_odom.vy,m_data_odom.vthx,m_data_odom.vthz);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: CalcWithSpeed
 * detail: Use speed data to calculate location
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFuncOdom::CalcWithSpeed(double v_x,double v_y,double v_r,double time_now,int serial_num,unsigned char rotate_mode)
{
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    static double time_last = 0.0,dt = 0.0;
    static int serial_num_last = 0;
    //set zero
    if(!m_set_zero)
    {
      time_last = time_now;
      serial_num_last = serial_num;
      m_set_zero = true;
    }
    //dt = time_now - time_last;
    dt = (serial_num - serial_num_last)*0.02;
    if(dt < 0.0)dt+=10000;
    if(dt == 0.0)return;
    serial_num_last = serial_num;
    time_last = time_now;
    if(m_model.compare("Differential")==0 || m_model.compare("Mecanum")==0)
    {
        double delta_th = v_r * dt;
        double delta_x = (v_x * cos(delta_th) - v_y * sin(delta_th)) * dt;
        double delta_y = (v_x * sin(delta_th) + v_y * cos(delta_th)) * dt;

        m_data_odom.x += cos(m_data_odom.th) * delta_x - sin(m_data_odom.th) * delta_y;
        m_data_odom.y += sin(m_data_odom.th) * delta_x + cos(m_data_odom.th) * delta_y;
        m_data_odom.th += delta_th;

        m_data_odom.vx = v_x;
        m_data_odom.vy = v_y;
        m_data_odom.vthz = v_r;
    }
    //////////////////////////////////////////////////////////////////////////////////////////////
    if(m_model.compare("Ackermann")==0)
    {
      if(m_wheel_base == 999.0)
        {
          ros_interface.ROSLog(LogLevel::kError," xnode_vehicle: Unknown WHEEL BASE base for odom calculation");
          ros_interface.Shutdown();
          return;
        }
        double delta_th = asin(v_x * dt * tan(v_r) / m_wheel_base);
        double delta_x = v_x * cos(delta_th) * dt;
        double delta_y = v_x * sin(delta_th) * dt;
        //ros_interface.ROSLog(LogLevel::kError," %f %f",v_r/3.14*180 ,m_wheel_base /tan(v_r));
        m_data_odom.x += cos(m_data_odom.th) * delta_x - sin(m_data_odom.th) * delta_y;
        m_data_odom.y += sin(m_data_odom.th) * delta_x + cos(m_data_odom.th) * delta_y;
        m_data_odom.th += delta_th;

        m_data_odom.vx = v_x;
        m_data_odom.vy = 0;
        m_data_odom.vthz = delta_th/dt;
        m_data_odom.vthx = v_r;
    }
    /*/////////////////////////////////////////////////////////////////////////////////////////////
    if(m_model.compare("Ackermann-D/Differential")==0)
    {
        if(m_wheel_base == 999.0)
        {
          ros_interface.ROSLog(LogLevel::kError," xnode_vehicle: Unknown WHEEL BASE for odom calculation");
          ros_interface.Shutdown();
          return;
        }
        if(rotate_mode == MODE_ANGLE_VEL)
        {
            double delta_th = v_r * dt;
            double delta_x = v_x * cos(delta_th) * dt;
            double delta_y = v_x * sin(delta_th) * dt;

            m_data_odom.x += cos(m_data_odom.th) * delta_x - sin(m_data_odom.th) * delta_y;
            m_data_odom.y += sin(m_data_odom.th) * delta_x + cos(m_data_odom.th) * delta_y;
            m_data_odom.th += delta_th;

            m_data_odom.vx = v_x;
            m_data_odom.vy = 0;
            m_data_odom.vth = v_r;
        }
        if(rotate_mode == MODE_ANGLE)
        {
          double delta_th = atan(v_x * dt * tan(v_r)*2 / m_wheel_base);
          double delta_x = v_x * cos(delta_th) * dt;
          double delta_y = v_x * sin(delta_th) * dt;

          m_data_odom.x += cos(m_data_odom.th) * delta_x - sin(m_data_odom.th) * delta_y;
          m_data_odom.y += sin(m_data_odom.th) * delta_x + cos(m_data_odom.th) * delta_y;
          m_data_odom.th += delta_th;

          m_data_odom.vx = v_x;
          m_data_odom.vy = 0;
          m_data_odom.vth = delta_th/dt;
        }
    }*/

}
/*------------------------------------------------------------------------------------------------------------------
 * name: CalcWithOdom
 * detail: Use odom data to calculate location
 -----------------------------------------------------------------------------------------------------------------*/
void VehicleFuncOdom::CalcWithOdom(double v_r,double odom_l,double odom_r,double odom_l_s,double odom_r_s,double time_now,int serial_num,unsigned char rotate_mode)
{
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    static double time_last = 0.0,dt = 0.0,d_avg_last = 0.0,d_avg_y_last = 0.0,d_th_last = 0.0;
    static int serial_num_last = 0;
    rotate_mode = 0;
    //set zero
    if(!m_set_zero)
    {
      m_data_odom.odom_l = odom_l;
      m_data_odom.odom_r = odom_r;
      m_data_odom.odom_l_s = odom_l_s;
      m_data_odom.odom_r_s = odom_r_s;
      time_last = time_now;
      serial_num_last = serial_num;
      m_set_zero = true;
    }
    //dt = time_now - time_last;
    dt = (serial_num - serial_num_last)*0.02;
    if(dt < 0.0)dt+=10000;
    if(dt == 0.0)return;
    serial_num_last = serial_num;
    time_last = time_now;
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(m_model.compare("Differential")==0)
    {
        if(m_wheel_base == 999.0)
        {
            ros_interface.ROSLog(LogLevel::kError," xnode_vehicle: Unknown WHEEL BASE for odom calculation");
            ros_interface.Shutdown();
            return;
        }
        if(m_track_width == 999.0)
        {
            ros_interface.ROSLog(LogLevel::kError," xnode_vehicle: Unknown TRACK WIDTH for odom calculation");
            ros_interface.Shutdown();
            return;
        }
        static double math_buff = pow(sqrt(pow(m_track_width,2) + pow(m_wheel_base,2)),2);
        double d_odom_l =odom_l-m_data_odom.odom_l;
        double d_odom_r =odom_r-m_data_odom.odom_r;
        double d_avg = (d_odom_r + d_odom_l) / 2.0;
        double d_th = (d_odom_r - d_odom_l)*m_track_width / math_buff;


        d_avg = (d_avg_last + d_avg)/2;
        if(d_avg < 0.0005 && d_avg > -0.0005)d_avg = 0;
        d_avg_last = d_avg;

        d_th = (d_th_last + d_th)/2;
        if(d_th < 0.001 && d_th > -0.001)d_th = 0;
        d_th_last = d_th;

        m_data_odom.odom_l = odom_l;
        m_data_odom.odom_r = odom_r;
        m_data_odom.vx = d_avg / dt;
        m_data_odom.vy = 0;
        m_data_odom.vthz = d_th / dt;
        
        double delta_x = cos(d_th) * d_avg;
        double delta_y = -sin(d_th) * d_avg;

        m_data_odom.x += cos(m_data_odom.th) * delta_x - sin(m_data_odom.th) * delta_y;
        m_data_odom.y += sin(m_data_odom.th) * delta_x + cos(m_data_odom.th) * delta_y;
        m_data_odom.th += d_th;
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(m_model.compare("Ackermann")==0)
    {
        if(m_wheel_base == 999.0)
        {
            ros_interface.ROSLog(LogLevel::kError," xnode_vehicle: Unknown WHEEL BASE for odom calculation");
            ros_interface.Shutdown();
            return;
        }
        double d_odom_l =odom_l-m_data_odom.odom_l;
        double d_odom_r =odom_r-m_data_odom.odom_r;
        double d_avg = (d_odom_r + d_odom_l) / 2.0;
        double d_th = (d_odom_r - d_odom_l)/ m_track_width;
        m_data_odom.odom_l = odom_l;
        m_data_odom.odom_r = odom_r;

        d_avg = (d_avg_last + d_avg)/2;
        if(d_avg < 0.0005 && d_avg > -0.0005)d_avg = 0;
        d_avg_last = d_avg;

        d_th = (d_th_last + d_th)/2;
        if(d_th < 0.001 && d_th > -0.001)d_th = 0;
        d_th_last = d_th;

        m_data_odom.vx = d_avg / dt;
        m_data_odom.vy = 0;
        m_data_odom.vthz = d_th / dt;
        m_data_odom.vthx = v_r;
        //ros_interface.ROSLog(LogLevel::kError," %f %f",v_r/3.14*180 ,m_wheel_base /tan(v_r));
        double delta_x = cos(d_th) * d_avg;
        double delta_y = -sin(d_th) * d_avg;

        m_data_odom.x += cos(m_data_odom.th) * delta_x - sin(m_data_odom.th) * delta_y;
        m_data_odom.y += sin(m_data_odom.th) * delta_x + cos(m_data_odom.th) * delta_y;
        m_data_odom.th += d_th;
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(m_model.compare("Mecanum")==0)
    {
      if(m_wheel_base == 999.0)
      {
          ros_interface.ROSLog(LogLevel::kError," xnode_vehicle: Unknown WHEEL BASE for odom calculation");
          ros_interface.Shutdown();
          return;
      }
      if(m_track_width == 999.0)
      {
          ros_interface.ROSLog(LogLevel::kError," xnode_vehicle: Unknown TRACK WIDTH for odom calculation");
          ros_interface.Shutdown();
          return;
      }
      double d_odom_l =odom_l-m_data_odom.odom_l;
      double d_odom_r =odom_r-m_data_odom.odom_r;
      double d_odom_l_s =odom_l_s-m_data_odom.odom_l_s;
      double d_odom_r_s =odom_r_s-m_data_odom.odom_r_s;
      m_data_odom.odom_l = odom_l;
      m_data_odom.odom_r = odom_r;
      m_data_odom.odom_l_s = odom_l_s;
      m_data_odom.odom_r_s = odom_r_s;

      double d_avg_x = (d_odom_r_s + d_odom_r + d_odom_l_s + d_odom_l) / 4;
      double d_avg_y = (d_odom_r + d_odom_l_s - d_odom_l - d_odom_r_s) / 4;
      double d_th = (d_odom_r_s + d_odom_r - d_odom_l - d_odom_l_s) / 2 / (m_track_width + m_wheel_base);  

      d_avg_x = (d_avg_last + d_avg_x)/2;
      if(d_avg_x < 0.0005 && d_avg_x > -0.0005)d_avg_x = 0;
      d_avg_last = d_avg_x;

      d_avg_y = (d_avg_y_last + d_avg_y)/2;
      if(d_avg_y < 0.0005 && d_avg_y > -0.0005)d_avg_y = 0;
      d_avg_y_last = d_avg_y;

      d_th = (d_th_last + d_th)/2;
      if(d_th < 0.001 && d_th > -0.001)d_th = 0;
      d_th_last = d_th;

      m_data_odom.vx = d_avg_x / dt;
      m_data_odom.vy = d_avg_y / dt;
      m_data_odom.vthz = d_th / dt;

      double delta_x = d_avg_x * cos(d_th) - d_avg_y * sin(d_th);
      double delta_y = d_avg_x * sin(d_th) + d_avg_y * cos(d_th);

      m_data_odom.x += cos(m_data_odom.th) * delta_x - sin(m_data_odom.th) * delta_y;
      m_data_odom.y += sin(m_data_odom.th) * delta_x + cos(m_data_odom.th) * delta_y;
      m_data_odom.th += d_th;

    }
    /*////////////////////////////////////////////////////////////////////////////////////////////////////////
    if(m_model.compare("Ackermann-D/Differential")==0)
    {
        if(m_wheel_base == 999.0)
        {
            ros_interface.ROSLog(LogLevel::kError," xnode_vehicle: Unknown WHEEL BASE for odom calculation");
            ros_interface.Shutdown();
            return;
        }
        if(m_track_width == 999.0)
        {
            ros_interface.ROSLog(LogLevel::kError," xnode_vehicle: Unknown TRACK WIDTH for odom calculation");
            ros_interface.Shutdown();
            return;
        }
        if(rotate_mode == MODE_ANGLE_VEL)
        {
            static double math_buff = sqrt(pow(m_track_width,2) + pow(m_wheel_base,2));
            double d_odom_l =odom_l-m_data_odom.odom_l;
            double d_odom_r =odom_r-m_data_odom.odom_r;
            double d_avg = (d_odom_r + d_odom_l) / 2.0;
            double d_th = (d_odom_r - d_odom_l) / math_buff;
            m_data_odom.odom_l = odom_l;
            m_data_odom.odom_r = odom_r;

            m_data_odom.vx = d_avg / dt;
            m_data_odom.vy = 0;
            m_data_odom.vth = d_th / dt;

            double delta_x = cos(d_th) * d_avg;
            double delta_y = -sin(d_th) * d_avg;


            m_data_odom.x += cos(m_data_odom.th) * delta_x - sin(m_data_odom.th) * delta_y;
            m_data_odom.y += sin(m_data_odom.th) * delta_x + cos(m_data_odom.th) * delta_y;
            m_data_odom.th += d_th;
        }
        if(rotate_mode == MODE_ANGLE)
        {

        }
    }*/

}
