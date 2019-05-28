// Autonomy ROS node, which is also a PLEXIL application.

// Under construction!

#include <ros/ros.h>
#include <owatb_interface/CartesianGuardedMove.h>
#include <ow_lander/StartPlanning.h>
#include "ExecApplication.hh"

using namespace std;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "autonomy_node");

  ros::NodeHandle n;

  ros::ServiceClient client =
    n.serviceClient<ow_lander::StartPlanning>("StartPlanning");
  ow_lander::StartPlanning srv;
  srv.request.use_defaults = true;
  srv.request.trench_x = 0.0;
  srv.request.trench_y = 0.0;
  srv.request.trench_d = 0.0;
  srv.request.delete_prev_traj = false;

  if (client.call(srv)) {
    ROS_INFO("StartPlanning returned: %d, %s",
             srv.response.success,
             srv.response.message.c_str());
  }
  else {
    ROS_ERROR("Failed to call service StartPlanning");
    return 1;
  }

  ros::Rate rate(1);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
