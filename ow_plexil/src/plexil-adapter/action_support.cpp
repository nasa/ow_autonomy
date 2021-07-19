#include "action_support.h"

t_action_active_cb default_action_active_cb (const std::string& operation_name)
{
  return [&] () { ROS_INFO ("%s started...", operation_name.c_str()); };
}
