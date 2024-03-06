//////////////////////////////////////////////////
// xpkg_template
//////////////////////////////////////////////////
#include <ros_interface.h>
#include <template_func.h>

using namespace XROS_TEMPLATE;

void TimeCallback() {
  enum class FiniteState { kInitState = 0, kWorkState };
  static FiniteState finite_state_machine_state = FiniteState::kInitState;
  static TemplateFunc& template_func = TemplateFunc::GetTemplateFunc();
  static ROSInterface& ros_interface = ROSInterface::GetInterface();

  switch (finite_state_machine_state) {
    case FiniteState::kInitState: {
      if (template_func.Init()) {
        ros_interface.ROSLog(LogLevel::kInfo, "%s : Init Succeded",
                             "template");
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        ros_interface.ROSLog(LogLevel::kWarn, "%s : Init Failed",
                             "template");
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    case FiniteState::kWorkState: {
      if (template_func.Work()) {
        // ros_interface.ROSLog(LogLevel::kInfo, "%s : Work Succeded",
        //                    "template");
        finite_state_machine_state = FiniteState::kWorkState;
      } else {
        ros_interface.ROSLog(LogLevel::kWarn, "%s : Work Failed", "template");
        finite_state_machine_state = FiniteState::kInitState;
      }
      break;
    }
    default: {
      ros_interface.ROSLog(LogLevel::kError, "%s : Unknown State", "template");
      finite_state_machine_state = FiniteState::kInitState;
      break;
    }
  }
}

/*------------------------------------------------------------------------------------------------------------------
 * name: main
 -----------------------------------------------------------------------------------------------------------------*/
int main(int argc, char** argv) {
  ROSInterface& ros_interface = ROSInterface::GetInterface();
  ros_interface.BaseInit(argc, argv, "xnode_template", 20.0, TimeCallback);
  ros_interface.NodeCheck("xnode_comm");

  ros_interface.Work();
  ros_interface.BaseDeinit();
  return 0;
}