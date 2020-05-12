// Temporary file.  An experimental dummy action client for MoveGuarded. 

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ow_autonomy/MoveGuardedAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "MoveGuardedClient");

  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<ow_autonomy::MoveGuardedAction>
    client ("MoveGuarded", true);

  ROS_INFO("Waiting for MoveGuarded action server to start...");
  client.waitForServer(); // will wait indefinitely

  ROS_INFO("MoveGuarded action server available, sending goal.");
  ow_autonomy::MoveGuardedGoal goal;
  goal.use_defaults = true;
  /*
  goal.delete_prev_traj = true;
  goal.target_x = 0;
  goal.target_y = 0;
  goal.target_z = 0;
  goal.surface_normal_x = 0;
  goal.surface_normal_y = 0;
  goal.surface_normal_z = 0;
  goal.offset_distance = 0;
  goal.overdrive_distance = 0;
  goal.retract = 0;
  */
  client.sendGoal (goal);

  // Wait for the action to return
  bool finished_before_timeout = client.waitForResult (ros::Duration(30.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = client.getState();
    ROS_INFO("MoveGuarded action finished: %s", state.toString().c_str());
    ow_autonomy::MoveGuardedResultConstPtr result = client.getResult();
    ROS_INFO("Action result message: %s", result->message.c_str());
  }
  else {
    ROS_INFO("MoveGuarded action did not finish before the time out.");
  }
  return 0;
}
