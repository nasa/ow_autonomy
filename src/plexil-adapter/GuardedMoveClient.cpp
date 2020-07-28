// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// Temporary file.  An experimental dummy action client for GuardedMove.

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <ow_autonomy/GuardedMoveAction.h>
#include <thread>
using std::thread;

static void spinThread()
{
  ros::spin();
}

static void doneCB (const actionlib::SimpleClientGoalState& state,
                    const ow_autonomy::GuardedMoveResultConstPtr& result)
{
  ROS_INFO ("Finished in state [%s]", state.toString().c_str());
}

static void activeCB()
{
  ROS_INFO ("Goal just went active");
}

static void feedbackCB (const ow_autonomy::GuardedMoveFeedbackConstPtr& feedback)
{
  ROS_INFO ("Feedback: (%f, %f, %f)",
            feedback->current_x, feedback->current_y, feedback->current_z);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "GuardedMoveClient");

  // second argument tells client whether or not to spin its own thread
  actionlib::SimpleActionClient<ow_autonomy::GuardedMoveAction>
    client ("GuardedMove", true);

  // We don't need this thread, because above we have client spin its own.
  //  thread node_thread (spinThread);

  ROS_INFO("Waiting for GuardedMove action server to start...");
  client.waitForServer(); // will wait indefinitely

  ROS_INFO("GuardedMove action server available, sending goal.");
  ow_autonomy::GuardedMoveGoal goal;
  goal.use_defaults = false;
  goal.delete_prev_traj = false;
  goal.target_x = 1.2;
  goal.target_y = 0.8;
  goal.target_z = 0.3;
  goal.direction_x = 0;
  goal.direction_y = 0;
  goal.direction_z = 0;
  goal.search_distance = 0;
  client.sendGoal (goal, &doneCB, &activeCB, &feedbackCB);

  // Wait for the action to return
  bool finished_before_timeout = client.waitForResult (ros::Duration(30.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = client.getState();
    ROS_INFO("GuardedMove action finished: %s", state.toString().c_str());
    ow_autonomy::GuardedMoveResultConstPtr result = client.getResult();
    ROS_INFO("GuardedMove action result: (%f, %f, %f)",
             result->final_x, result->final_y, result->final_z);
  }
  else {
    ROS_INFO("GuardedMove action did not finish before the time out.");
  }

  ros::shutdown();

  // The following is needed only if we spin our own thread above.
  //  node_thread.join();

  return 0;
}
