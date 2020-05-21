// Temporary file.  A dummy action server for MoveGuarded.

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ow_autonomy/MoveGuardedAction.h>
#include <string>

using std::string;

class MoveGuardedAction
{
 public:

  MoveGuardedAction (const string& name)
    :
    // Originally used the simpler execute callback.
    //    m_actionServer (m_nodeHandle, name,
    //                    boost::bind (&MoveGuardedAction::executeCB, this, _1),
    //                    false),
    m_actionServer (m_nodeHandle, name, false),
    m_actionName (name)
  {
    m_actionServer.registerGoalCallback
      (boost::bind (&MoveGuardedAction::goalCB, this));
    m_actionServer.registerPreemptCallback
      (boost::bind (&MoveGuardedAction::preemptCB, this));
    m_actionServer.start();
  }

  ~MoveGuardedAction () = default;
  MoveGuardedAction (const MoveGuardedAction&) = delete;
  MoveGuardedAction& operator= (const MoveGuardedAction&) = delete;

  // Used to perform action, but not as a callback.
  void executeCB (const ow_autonomy::MoveGuardedGoalConstPtr& goal)
  {
    ros::Rate r(1);
    bool success = true;
    int iterations = 10; // one per second
    float x_incr = goal->target_x / iterations;
    float y_incr = goal->target_y / iterations;
    float z_incr = goal->target_z / iterations;

    m_feedback.current_x = m_feedback.current_y = m_feedback.current_z = 0;
    ROS_INFO ("%s: Executing", m_actionName.c_str());

    // start executing the action
    for (int i = 0; i <= iterations; i++) {
      if (m_actionServer.isPreemptRequested() || !ros::ok()) {
        ROS_INFO ("%s: Preempted", m_actionName.c_str());
        m_actionServer.setPreempted();
        success = false;
        break;
      }
      m_feedback.current_x += x_incr;
      m_feedback.current_y += y_incr;
      m_feedback.current_z += z_incr;
      m_actionServer.publishFeedback (m_feedback);
      r.sleep();
    }

    if (success) {
      m_result.final_x = goal->target_x;
      m_result.final_y = goal->target_y;
      m_result.final_z = goal->target_z;
      ROS_INFO ("%s: Succeeded", m_actionName.c_str());
      m_actionServer.setSucceeded (m_result);
    }
  }

  void goalCB ()
  {
    ROS_INFO ("%s: New goal available.", m_actionName.c_str());
    auto goal = m_actionServer.acceptNewGoal();

    // NOTE: this was originally registered as a callback. Now calling it
    // directly for convenience, and decided not to change its name.
    executeCB (goal);
  }

  void preemptCB ()
  {
    ROS_INFO ("%s: Preempted.", m_actionName.c_str());
    m_actionServer.setPreempted();
  }

 private:
  ros::NodeHandle m_nodeHandle;
  actionlib::SimpleActionServer<ow_autonomy::MoveGuardedAction> m_actionServer;
  string m_actionName;
  ow_autonomy::MoveGuardedFeedback m_feedback;
  ow_autonomy::MoveGuardedResult m_result;
};


int main (int argc, char** argv)
{
  ros::init(argc, argv, "MoveGuardedServer");
  MoveGuardedAction MoveGuarded ("MoveGuarded");
  ros::spin();
  return 0;
}
