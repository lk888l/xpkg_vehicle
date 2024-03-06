/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui blank@invalid.example
 * Date   : 2023-10-31
 ****************************************************************/
#include <template_func.h>

using namespace XROS_TEMPLATE;

bool TemplateFunc::Init() {
  static ROSInterface& ros_interface = ROSInterface::GetInterface();
  m_out_str = ros_interface.m_out_str;
  m_in_str = "";

  return true;
}

bool TemplateFunc::Work() {
  static ROSInterface& ros_interface = ROSInterface::GetInterface();

  if (ros_interface.GetInStrFlag()) {
    m_in_str = ros_interface.GetInStr();
    ros_interface.ResetInStrFlag();

    ros_interface.ROSLog(LogLevel::kInfo, "I hear: %s", m_in_str.c_str());
  }

  ros_interface.PubOutStr(m_out_str);

  return true;
}