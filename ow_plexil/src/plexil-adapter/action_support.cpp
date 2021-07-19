#include "action_support.h"

std::function<void()> default_action_active_cb (const std::string& operation_name)
{
  return [&] () { ROS_INFO ("%s started...", operation_name.c_str()); };
}
