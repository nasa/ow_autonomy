// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <functional>
#include "OWLATSimInterface.h"

using std::hash;
using std::string;

const string Name_OwlatUnstow = "/owlat_sim/ARM_UNSTOW";
constexpr auto Id_OwlatUnstow = hash<string>(Name_OwlatUnstow);

void OWLATSimInterface::initialize()
{
  registerLanderOperation (Name_OwlatUnstow);
    
  if (! m_owlatUnstowClient->waitForServer(ros::Duration(ActionServerTimeout))) {
    ROS_ERROR ("OWLAT Unstow action server did not connect!");
  }
}

/////////////////////////////// OWLAT Interface ////////////////////////////////

void OWLATSimInterface::owlatUnstow (int id)
{
  if (! markOperationRunning (Name_OwlatUnstow, id)) return;
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
    (Name_OwlatUnstow, m_owlatUnstowClient, goal, id);
}
