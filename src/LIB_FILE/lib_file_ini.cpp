#include <lib_file_ini.h>
#include <ros_interface.h>
using namespace XROS_VEHICLE;

LibFileIni::LibFileIni()
{
    m_fp = nullptr;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: GetVehicleFuncOdom
 -----------------------------------------------------------------------------------------------------------------*/
LibFileIni& LibFileIni::GetLibFileIni()
{
  static LibFileIni lib_file_ini;
  return lib_file_ini;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: OpenFile
 -----------------------------------------------------------------------------------------------------------------*/
bool LibFileIni::OpenFile(const char* path_name)
{
    string file_data,section,key,key_value;
    ROSInterface& ros_interface = ROSInterface::GetInterface();
    char temp_data[1024] = {0};
    KEYMAP m_last_map;
    unsigned long  pos_p = 0,pos_l = 0,pos_r = 0;


    m_fp = fopen(path_name, "r");
    if (m_fp == nullptr)
    {
         ros_interface.ROSLog(LogLevel::kError,"open ini file %s error!",path_name);
        return false;
    }

    m_key_map_main.clear();

    while(fgets(temp_data,1024,m_fp))
    {
        file_data.assign(temp_data);
        //delete no need char//////////////////////////////
        pos_l = file_data.find("#");
        if(string::npos != pos_l)continue;
        pos_l = file_data.find("\n");
        if(string::npos != pos_l)file_data.erase(pos_l,1);
        pos_l = file_data.find("\r" );
        if(string::npos != pos_l)file_data.erase(pos_l,1);
        //if is section////////////////////////////////////
        pos_l = file_data.find("[");
        pos_r = file_data.find("]");
        if(pos_l != string::npos && pos_r != string::npos)
        {
            file_data.erase(pos_l,1);
            pos_r--;
            file_data.erase(pos_r,1);
            m_key_map_main[section] = m_last_map;
            m_last_map.clear();
            section = file_data;
        }
        else
        {
            //if is key////////////////////////////////////
            pos_p = file_data.find("=");
            if(pos_p != string::npos)
            {
                key = file_data.substr(0,pos_p);
                key_value = file_data.substr(pos_p+1,file_data.length()-pos_p-1);
                m_last_map[key] = key_value ;
            }
        }
    }
    m_key_map_main[section] = m_last_map;
    fclose(m_fp);
    return true;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: GetKey
 -----------------------------------------------------------------------------------------------------------------*/
string LibFileIni::GetKeyValue(const char* section, const char* key)
{
    KEYMAP key_map = m_key_map_main[section];
    string temp = key_map[key];
    return temp;
}
/*------------------------------------------------------------------------------------------------------------------
 * name: GetKeyInt
 -----------------------------------------------------------------------------------------------------------------*/
int LibFileIni::GetKeyInt(const char* section, const char* key)
{
    string temp = GetKeyValue(section,key);
    if(temp.size() == 0)return 0;
    return stoi(temp);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: GetKeyDouble
 -----------------------------------------------------------------------------------------------------------------*/
double LibFileIni::GetKeyDouble(const char* section, const char* key)
{
    string temp = GetKeyValue(section,key);
    if(temp.size() == 0)return 0;
    return stod(temp);
}
/*------------------------------------------------------------------------------------------------------------------
 * name: ToHexStr
 -----------------------------------------------------------------------------------------------------------------*/
string LibFileIni::ToHexStr(int num)
{
    char temp[80];
    sprintf(temp,"0x%02X",num);
    string str = temp;
    return str;
}
