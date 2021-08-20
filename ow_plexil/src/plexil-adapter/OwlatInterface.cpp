// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <thread>
#include <vector>
#include "OwlatInterface.h"

using namespace owlat_sim_msgs;
using std::hash;
using std::string;
using std::thread;
using std::string;
using std::vector;

const string Name_OwlatUnstow = "/owlat_sim/ARM_UNSTOW";
const string Name_OwlatStow =   "/owlat_sim/ARM_STOW";
const string Name_OwlatArmMoveCartesian =   "/owlat_sim/ARM_MOVE_CARTESIAN";

// Used as indices into the subsequent vector.
enum LanderOps {
  OwlatUnstow,
  OwlatStow,
  OwlatArmMoveCartesian
};

static std::vector<string> LanderOpNames =
  { Name_OwlatUnstow,
    Name_OwlatStow,
    Name_OwlatArmMoveCartesian
  };


std::shared_ptr<OwlatInterface> OwlatInterface::m_instance = nullptr;

std::shared_ptr<OwlatInterface> OwlatInterface::instance ()
{
  // Very simple singleton
  if (m_instance == nullptr) m_instance = std::make_shared<OwlatInterface>();
  return m_instance;
}

void OwlatInterface::initialize()
{
  static bool initialized = false;

  if (not initialized) {

    for (auto name : LanderOpNames) {
      registerLanderOperation (name);
    }

    // Initialize pointers
    m_owlatUnstowClient =
      std::make_unique<OwlatUnstowActionClient>(Name_OwlatUnstow, true);
    m_owlatStowClient =
      std::make_unique<OwlatStowActionClient>(Name_OwlatStow, true);
    m_owlatArmMoveCartesianClient =
      std::make_unique<OwlatArmMoveCartesianActionClient>(Name_OwlatArmMoveCartesian, true);


    // Connect to action servers
    if (! m_owlatUnstowClient->waitForServer(ros::Duration(ActionServerTimeout))) {
      ROS_ERROR ("OWLAT Unstow action server did not connect!");
    }
    if (! m_owlatStowClient->waitForServer(ros::Duration(ActionServerTimeout))) {
      ROS_ERROR ("OWLAT Stow action server did not connect!");
    }
    if (! m_owlatArmMoveCartesianClient->waitForServer(ros::Duration(ActionServerTimeout))) {
      ROS_ERROR ("OWLAT ARM_MOVE_CARTESIAN action server did not connect!");
    }

  }
}


/////////////////////////////// OWLAT Interface ////////////////////////////////

void OwlatInterface::owlatUnstow (int id)
{
  if (! markOperationRunning (Name_OwlatUnstow, id)) return;
  thread action_thread (&OwlatInterface::owlatUnstowAction, this, id);
  action_thread.detach();
}

void OwlatInterface::owlatUnstowAction (int id)
{
  ARM_UNSTOWGoal goal;
  string opname = Name_OwlatUnstow;  // shorter version

  runAction<actionlib::SimpleActionClient<ARM_UNSTOWAction>,
            ARM_UNSTOWGoal,
            ARM_UNSTOWResultConstPtr,
            ARM_UNSTOWFeedbackConstPtr>
    (opname, m_owlatUnstowClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ARM_UNSTOWFeedbackConstPtr> (opname),
     default_action_done_cb<ARM_UNSTOWResultConstPtr> (opname));
}

void OwlatInterface::owlatStow (int id)
{
  if (! markOperationRunning (Name_OwlatStow, id)) return;
  thread action_thread (&OwlatInterface::owlatStowAction, this, id);
  action_thread.detach();
}

void OwlatInterface::owlatStowAction (int id)
{
  ARM_STOWGoal goal;
  string opname = Name_OwlatStow;  // shorter version

  runAction<actionlib::SimpleActionClient<ARM_STOWAction>,
            ARM_STOWGoal,
            ARM_STOWResultConstPtr,
            ARM_STOWFeedbackConstPtr>
    (opname, m_owlatStowClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ARM_STOWFeedbackConstPtr> (opname),
     default_action_done_cb<ARM_STOWResultConstPtr> (opname));
}

void OwlatInterface::owlatArmMoveCartesian (int frame, bool relative, 
                                            vector<double> position, 
                                            vector<double> orientation,
                                            int id)
{
  if (! markOperationRunning (Name_OwlatArmMoveCartesian, id)) return;
  thread action_thread (&OwlatInterface::owlatArmMoveCartesianAction, this,
                        frame, relative, position, orientation, id);
  action_thread.detach();
}

void OwlatInterface::owlatArmMoveCartesianAction (int frame, bool relative, 
                                            vector<double> position, 
                                            vector<double> orientation,
                                            int id)
{
  ARM_MOVE_CARTESIANGoal goal;
  goal.frame.value = frame;
  goal.relative = relative;
  goal.pose.position.x = position[0];
  goal.pose.position.y = position[1];
  goal.pose.position.z = position[2];
  goal.pose.orientation.x = orientation[0];
  goal.pose.orientation.y = orientation[1];
  goal.pose.orientation.z = orientation[2];
  goal.pose.orientation.w = orientation[3];
  string opname = Name_OwlatArmMoveCartesian;  // shorter version

  runAction<actionlib::SimpleActionClient<ARM_MOVE_CARTESIANAction>,
            ARM_MOVE_CARTESIANGoal,
            ARM_MOVE_CARTESIANResultConstPtr,
            ARM_MOVE_CARTESIANFeedbackConstPtr>
    (opname, m_owlatArmMoveCartesianClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ARM_MOVE_CARTESIANFeedbackConstPtr> (opname),
     default_action_done_cb<ARM_MOVE_CARTESIANResultConstPtr> (opname));
}
