#include "OwHardware.h"
//#include <std_msgs/Float64MultiArray>

OwHardware::OwHardware ()
  : m_armMoveCartesianClient ("ARM_MOVE_CARTESIAN", true)
{ }


// Action support

static void arm_move_cartesian_done_cb
(const actionlib::SimpleClientGoalState& state,
 const owlat_msgs::ARM_MOVE_CARTESIANResultConstPtr& result)
{
  ROS_INFO ("ARM_MOVE_CARTESIAN done callback: finished in state [%s]",
            state.toString().c_str());
  ROS_INFO ("ARM_MOVE_CARTESIAN done callback: result ([%f, %f, %f], %f)",
            result->position[0], result->position[1], result->position[2],
            result->distance);
}

static void arm_move_cartesian_active_cb ()
{
  ROS_INFO ("ARM_MOVE_CARTESIAN active callback - goal active!");
}

static void arm_move_cartesian_feedback_cb
(const owlat_msgs::ARM_MOVE_CARTESIANFeedbackConstPtr& feedback)
{
  ROS_INFO ("ARM_MOVE_CARTESIAN feedback callback: ([%f, %f, %f], %f)",
            feedback->final_position[0],
            feedback->final_position[1],
            feedback->final_position[2],
            feedback->final_distance);
}


void OwHardware::cartesianArmMove (bool relative,
                                   double pos_x, double pos_y, double pos_z,
                                   double norm_x, double norm_y, double norm_z,
                                   double distance, double overdrive, int id)
{
//  std_msgs::Float64MultiArray array_msg;
//  array_msg.data.resize(9);
  owlat_msgs::ARM_MOVE_CARTESIANGoal goal;

//  thread fault_thread (monitor_for_faults, Op_ARM_MOVE_CARTESIANAction);
  m_armMoveCartesianClient.sendGoal (goal,
                                     arm_move_cartesian_done_cb,
                                     arm_move_cartesian_active_cb,
                                     arm_move_cartesian_feedback_cb);

  // Wait for the action to return
  bool finished_before_timeout =
    m_armMoveCartesianClient.waitForResult (ros::Duration (30.0));

  if (finished_before_timeout) {
    actionlib::SimpleClientGoalState state = m_armMoveCartesianClient.getState();
    ROS_INFO("ARM_MOVE_CARTESIAN action finished: %s", state.toString().c_str());
    owlat_msgs::ARM_MOVE_CARTESIANResultConstPtr result =
      m_armMoveCartesianClient.getResult();
    ROS_INFO("ARM_MOVE_CARTESIAN action result: ()");

  }
  else {
    ROS_INFO("ARM_MOVE_CARTESIAN action did not finish before the time out.");
  }

//  mark_operation_finished (Op_ARM_MOVE_CARTESIANAction, id);
//  fault_thread.join();
}
