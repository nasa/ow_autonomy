// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "OWLATSimInterface.h"

const string Op_OwlatUnstow       = "/owlat_sim/ARM_UNSTOW";


// Both the following duplicated in OwInterface.cpp.  Needs refactoring or
// different representation.  Each operation name needs to be mapped to a unique
// int.

enum LanderOps {
  OwlatUnstow
};

static std::vector<string> LanderOpNames =
  {
    Op_OwlatUnstow
  };


void OWLATSimInterface::initialize()
{
  if (! m_owlatUnstowClient->waitForServer(ros::Duration(ActionServerTimeout))) {
    ROS_ERROR ("OWLAT Unstow action server did not connect!");
  }
}



/* Defined NewClass.  These values need initializing.
static map<string, int> Running
{
  { Op_OwlatUnstow, IDLE_ID }
};
*/


/////////////////////////////// OWLAT Interface ////////////////////////////////

void OWLATSimInterface::owlatUnstow (int id)
{
  if (! mark_operation_running (Op_OwlatUnstow, id)) return;
  thread action_thread (&OWLATSimInterface::owlatUnstowAction, this, id);
  action_thread.detach();
}

void OWLATSimInterface::owlatUnstowAction (int id)
{
  owlat_sim_msgs::ARM_UNSTOWGoal goal;

  runAction<OwlatUnstow,
            actionlib::SimpleActionClient<owlat_sim_msgs::ARM_UNSTOWAction>,
            owlat_sim_msgs::ARM_UNSTOWGoal,
            owlat_sim_msgs::ARM_UNSTOWResultConstPtr,
            owlat_sim_msgs::ARM_UNSTOWFeedbackConstPtr>
    (Op_OwlatUnstow, m_owlatUnstowClient, goal, id);
}
