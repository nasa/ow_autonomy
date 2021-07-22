// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include <thread>
#include <vector>
#include "OWLATSimInterface.h"

using namespace owlat_sim_msgs;
using std::hash;
using std::string;
using std::thread;

using std::string;

const string Name_OwlatUnstow = "/owlat_sim/ARM_UNSTOW";

// Used as indices into the subsequent vector.
enum LanderOps {
  OwlatUnstow
};

static std::vector<string> LanderOpNames =
  { Name_OwlatUnstow
  };


std::shared_ptr<OWLATSimInterface> OWLATSimInterface::m_instance = nullptr;

std::shared_ptr<OWLATSimInterface> OWLATSimInterface::instance ()
{
  // Very simple singleton
  if (m_instance == nullptr) m_instance = std::make_shared<OWLATSimInterface>();
  return m_instance;
}

void OWLATSimInterface::initialize()
{
  static bool initialized = false;

  if (not initialized) {

    for (auto name : LanderOpNames) {
      registerLanderOperation (name);
    }

    // Initialize pointers
    m_owlatUnstowClient =
      std::make_unique<OwlatUnstowActionClient>(Name_OwlatUnstow, true);

    // Connect to action servers
    if (! m_owlatUnstowClient->waitForServer(ros::Duration(ActionServerTimeout))) {
      ROS_ERROR ("OWLAT Unstow action server did not connect!");
    }
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
