/****************************************************************
 * Copyright 2023 Dong Zhaorui. All rights reserved.
 * Author : Dong Zhaorui blank@invalid.example
 * Date   : 2023-10-31
 ****************************************************************/
#ifndef FUNC_TEMPLATE_FUNC_H_
#define FUNC_TEMPLATE_FUNC_H_

#include <ros_interface.h>
#include <unistd.h>

#include <string>

using namespace std;
using namespace XROS_TEMPLATE;
// info of vehicle //////////////////////////////////////
class TemplateFunc {
 public:
  static TemplateFunc& GetTemplateFunc() {
    static TemplateFunc template_func;
    return template_func;
  }

  // Work Handle
  bool Init();
  bool Work();

 private:
  TemplateFunc() = default;
  virtual ~TemplateFunc() = default;

  std::string m_out_str;
  std::string m_in_str;
};

#endif  // FUNC_TEMPLATE_FUNC_H_
