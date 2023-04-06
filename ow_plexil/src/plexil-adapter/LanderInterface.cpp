// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
// Exploration Research and Simulation can be found in README.md in
// the root directory of this repository.

// ow_plexil
#include "LanderInterface.h"
#include "subscriber.h"

// PLEXIL
#include <ArrayImpl.hh>

// ROS
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// C++
#include <thread>
#include <vector>
#include <functional>

using namespace owlat_sim_msgs;
using namespace owl_msgs;
using namespace PLEXIL;

using std::hash;
using std::copy;
using std::string;
using std::thread;
using std::string;
using std::vector;
using std::make_unique;

const string Name_Unstow = "ArmUnstow";
const string Name_Stow = "ArmStow";
const string Name_ArmStop = "ArmStop";
const string Name_MoveJoint =     "ArmMoveJoint";
const string Name_ArmMoveCartesian = "ArmMoveCartesian";
const string Name_ArmMoveCartesianGuarded = "ArmMoveCartesianGuarded";

static vector<string> LanderOpNames = {
  Name_Discard,
  Name_Unstow,
  Name_Stow,
  Name_ArmMoveCartesian,
  Name_ArmMoveCartesianGuarded,
  Name_MoveJoint,
  Name_ArmStop,
};

LanderInterface* LanderInterface::instance ()
{
  // Very simple singleton
  static LanderInterface instance;
  return &instance;
}

void LanderInterface::initialize()
{
  static bool initialized = false;

  if (not initialized) {

    for (auto name : LanderOpNames) {
      registerLanderOperation (name);
    }

    m_genericNodeHandle = make_unique<ros::NodeHandle>();

    // Initialize action clients

    m_armFindSurfaceClient =
      make_unique<ArmFindSurfaceActionClient>(Op_ArmFindSurface, true);
    m_armStowClient =
      make_unique<ArmStowActionClient>(Name_Stow, true);
    m_armUnstowClient =
      make_unique<ArmUnstowActionClient>(Name_Unstow, true);
    m_taskDiscardClient =
      make_unique<TaskDiscardSampleActionClient>(Name_Discard, true);
    m_armMoveCartesianClient =
      make_unique<ArmMoveCartesianActionClient>(Name_ArmMoveCartesian,
                                                true);
    m_armMoveCartesianGuardedClient =
      make_unique<ArmMoveCartesianGuardedActionClient>
      (Name_ArmMoveCartesianGuarded, true);
    m_armMoveJointClient =
      make_unique<ArmMoveJointActionClient>(Name_MoveJoint, true);

    // Connect to action servers
    connectActionServer (m_armStopClient, Name_ArmStop, "/ArmStop/status");
    connectActionServer (m_armUnstowClient, Name_ArmUnstow, "/ArmUnstow/status");
    connectActionServer (m_armStowClient, Name_ArmStow, "/ArmStow/status");
    connectActionServer (m_armMoveJointClient, Name_ArmMoveJoint,
                         "/ArmMoveJoint/status");
    connectActionServer (m_armMoveCartesianClient, Op_ArmMoveCartesian,
                         "/ArmMoveCartesian/status");
    connectActionServer (m_armMoveCartesianGuardedClient, Op_ArmMoveCartesianGuarded,
                         "/ArmMoveCartesianGuarded/status");
    connectActionServer (m_armFindSurfaceClient, Op_ArmFindSurface,
                         "/ArmFindSurface/status");
  }
}

void LanderInterface::addSubscriber (const string& topic, const string& operation)
{
  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle -> subscribe<actionlib_msgs::GoalStatusArray>
      (topic, QSize,
       boost::bind(&OwInterface::actionGoalStatusCallback,
                   this, _1, operation))));
}


/////////////////////////////// Lander Interface ////////////////////////////////

void LanderInterface::armUnstow (int id)
{
  if (! markOperationRunning (Name_Unstow, id)) return;
  thread action_thread (&LanderInterface::nullaryAction<
                        ArmUnstowActionClient,
                        ArmUnstowGoal,
                        ArmUnstowResultConstPtr,
                        ArmUnstowFeedbackConstPtr>,
                        this, id, Name_Unstow, std::ref(m_armUnstowClient));
  action_thread.detach();
}

void LanderInterface::armStow (int id)
{
  if (! markOperationRunning (Name_Stow, id)) return;
  //  thread action_thread (&LanderInterface::armStowAction, this, id);
  thread action_thread (&LanderInterface::nullaryAction<
                        ArmStowActionClient,
                        ArmStowGoal,
                        ArmStowResultConstPtr,
                        ArmStowFeedbackConstPtr>,
                        this, id, Name_Stow, std::ref(m_armStowClient));
  action_thread.detach();
}

void LanderInterface::armMoveCartesian (int frame,
                                       bool relative,
                                       const vector<double>& position,
                                       const vector<double>& orientation,
                                       int id)
{
  if (! markOperationRunning (Name_ArmMoveCartesian, id)) return;

  geometry_msgs::Quaternion qm;

  // Deal with type of orientation
  if (orientation.size() == 3) { // assume Euler angle
    tf2::Quaternion q;
    // Yaw, pitch, roll, respectively.
    q.setEuler (orientation[2], orientation[1], orientation[0]);
    q.normalize();  // Recommended in ROS docs, not sure if needed here.
    qm = tf2::toMsg(q);
  }
  else { // assume quaternion orientation
    qm.x = orientation[0];
    qm.y = orientation[1];
    qm.z = orientation[2];
    qm.w = orientation[3];
  }

  geometry_msgs::Pose pose;
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];
  pose.orientation = qm;
  thread action_thread (&LanderInterface::armMoveCartesianAction, this,
                        frame, relative, pose, id);
  action_thread.detach();
}

void LanderInterface::armMoveCartesianAction (int frame,
                                             bool relative,
                                             const geometry_msgs::Pose& pose,
                                             int id)
{
  ArmMoveCartesianGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  goal.pose = pose;
  string opname = Name_ArmMoveCartesian;

  runAction<actionlib::SimpleActionClient<ArmMoveCartesianAction>,
            ArmMoveCartesianGoal,
            ArmMoveCartesianResultConstPtr,
            ArmMoveCartesianFeedbackConstPtr>
    (opname, m_armMoveCartesianClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ArmMoveCartesianFeedbackConstPtr> (opname),
     default_action_done_cb<ArmMoveCartesianResultConstPtr> (opname));
}

void LanderInterface::armMoveCartesianGuarded (int frame, bool relative,
                                              const vector<double>& position,
                                              const vector<double>& orientation,
                                              double force_threshold,
                                              double torque_threshold,int id)
{
  if (! markOperationRunning (Name_ArmMoveCartesianGuarded, id)) return;

  geometry_msgs::Quaternion qm;

  // Deal with type of orientation
  if (orientation.size() == 3) { // assume Euler angle
    tf2::Quaternion q;
    // Yaw, pitch, roll, respectively.
    q.setEuler (orientation[2], orientation[1], orientation[0]);
    q.normalize();  // Recommended in ROS docs, not sure if needed here.
    qm = tf2::toMsg(q);
  }
  else { // assume quaternion orientation
    qm.x = orientation[0];
    qm.y = orientation[1];
    qm.z = orientation[2];
    qm.w = orientation[3];
  }

  geometry_msgs::Pose pose;
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];
  pose.orientation = qm;
  thread action_thread (&LanderInterface::armMoveCartesianGuardedAction,
                        this, frame, relative, pose,
                        force_threshold, torque_threshold, id);
  action_thread.detach();
}

void LanderInterface::armMoveCartesianGuardedAction (int frame, bool relative,
                                                    const geometry_msgs::Pose& pose,
                                                    double force_threshold,
                                                    double torque_threshold,
                                                    int id)
{
  ArmMoveCartesianGuardedGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  goal.pose = pose;
  goal.force_threshold = force_threshold;
  goal.torque_threshold = torque_threshold;
  string opname = Name_ArmMoveCartesianGuarded;

  runAction<actionlib::SimpleActionClient<ArmMoveCartesianGuardedAction>,
            ArmMoveCartesianGuardedGoal,
            ArmMoveCartesianGuardedResultConstPtr,
            ArmMoveCartesianGuardedFeedbackConstPtr>
    (opname, m_armMoveCartesianGuardedClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ArmMoveCartesianGuardedFeedbackConstPtr> (opname),
     default_action_done_cb<ArmMoveCartesianGuardedResultConstPtr> (opname));
}

void LanderInterface::armMoveJoint (bool relative,
                                   int joint, double angle,
                                   int id)
{
  if (! markOperationRunning (Name_MoveJoint, id)) return;
  thread action_thread (&LanderInterface::armMoveJointAction,
                        this, relative, joint, angle, id);
  action_thread.detach();
}

void LanderInterface::armMoveJointAction (bool relative,
                                         int joint, double angle,
                                         int id)
{
  ArmMoveJointGoal goal;
  goal.relative = relative;
  goal.joint = joint;
  goal.angle = angle;
  string opname = Name_MoveJoint;  // shorter version

  runAction<actionlib::SimpleActionClient<ArmMoveJointAction>,
            ArmMoveJointGoal,
            ArmMoveJointResultConstPtr,
            ArmMoveJointFeedbackConstPtr>
    (opname, m_armMoveJointClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ArmMoveJointFeedbackConstPtr> (opname),
     default_action_done_cb<ArmMoveJointResultConstPtr> (opname));
}
