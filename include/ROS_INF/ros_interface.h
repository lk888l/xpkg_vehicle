#pragma once

#if X_ROS_VERSION==1
#include "internal_ros1_interface.h"
#elif X_ROS_VERSION==2
#include "internal_ros2_interface.h"
#else
#error X_ROS_VERSION is unknown
#endif
