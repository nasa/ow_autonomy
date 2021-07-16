#include "action_support.h"

std::function<void()> active_cb (const std::string& operation_name)
{
  return [&] () { ROS_INFO ("%s started...", operation_name.c_str()); };
}
