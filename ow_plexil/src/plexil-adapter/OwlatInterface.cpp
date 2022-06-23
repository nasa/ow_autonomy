// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
// Exploration Research and Simulation can be found in README.md in
// the root directory of this repository.

#include <thread>
#include <vector>
#include "OwlatInterface.h"
#include <ArrayImpl.hh>
#include <functional>
#include "subscriber.h"

using std::hash;
using std::copy;
using std::string;
using std::thread;
using std::string;
using std::vector;
using std::make_unique;
using namespace owlat_sim_msgs;
using namespace PLEXIL;

const string Name_OwlatUnstow =           "/owlat_sim/ARM_UNSTOW";
const string Name_OwlatStow =             "/owlat_sim/ARM_STOW";
const string Name_OwlatArmMoveCartesian = "/owlat_sim/ARM_MOVE_CARTESIAN";
const string Name_OwlatArmMoveCartesianGuarded =
  "/owlat_sim/ARM_MOVE_CARTESIAN_GUARDED";
const string Name_OwlatArmMoveJoint =     "/owlat_sim/ARM_MOVE_JOINT";
const string Name_OwlatArmMoveJoints =    "/owlat_sim/ARM_MOVE_JOINTS";
const string Name_OwlatArmMoveJointsGuarded =
  "/owlat_sim/ARM_MOVE_JOINTS_GUARDED";
const string Name_OwlatArmPlaceTool =     "/owlat_sim/ARM_PLACE_TOOL";
const string Name_OwlatArmSetTool =       "/owlat_sim/ARM_SET_TOOL";
const string Name_OwlatArmStop =          "/owlat_sim/ARM_STOP";
const string Name_OwlatArmTareFS =        "/owlat_sim/ARM_TARE_FS";
const string Name_OwlatTaskPSP =          "/owlat_sim/TASK_PSP";
const string Name_OwlatTaskScoop =        "/owlat_sim/TASK_SCOOP";

// Used as indices into the subsequent vector.
enum class LanderOps {
  OwlatUnstow,
  OwlatStow,
  OwlatArmMoveCartesian,
  OwlatArmMoveCartesianGuarded,
  OwlatArmMoveJoint,
  OwlatArmMoveJoints,
  OwlatArmMoveJointsGuarded,
  OwlatArmPlaceTool,
  OwlatArmSetTool,
  OwlatArmStop,
  OwlatArmTareFS,
  OwlatArmTaskPSP,
  OwlatArmTaskScoop
};

static vector<string> LanderOpNames = {
    Name_OwlatUnstow,
    Name_OwlatStow,
    Name_OwlatArmMoveCartesian,
    Name_OwlatArmMoveCartesianGuarded,
    Name_OwlatArmMoveJoint,
    Name_OwlatArmMoveJoints,
    Name_OwlatArmMoveJointsGuarded,
    Name_OwlatArmPlaceTool,
    Name_OwlatArmSetTool,
    Name_OwlatArmStop,
    Name_OwlatArmTareFS,
    Name_OwlatTaskPSP,
    Name_OwlatTaskScoop
  };

// Task PSP Callback
static double PSPStopReasonVar = 0;
template<int OpIndex, typename T>
static void task_psp_done_cb
(const actionlib::SimpleClientGoalState& state,
 const T& result)
{
  ROS_INFO("PSP Stop Reason: : %d", result->stop_reason.value);
  PSPStopReasonVar = result->stop_reason.value;
  publish("PSPStopReason", PSPStopReasonVar);
  ROS_INFO ("/owlat_sim/TASK_PSP finished in state %s",
            state.toString().c_str());
}

OwlatInterface* OwlatInterface::instance ()
{
  // Very simple singleton
  static OwlatInterface instance;
  return &instance;
}

void OwlatInterface::initialize()
{
  static bool initialized = false;

  if (not initialized) {

    for (auto name : LanderOpNames) {
      registerLanderOperation (name);
    }

    m_genericNodeHandle = make_unique<ros::NodeHandle>();
    m_arm_joint_angles.resize(7);
    m_arm_joint_accelerations.resize(7);
    m_arm_joint_torques.resize(7);
    m_arm_joint_velocities.resize(7);
    m_arm_ft_torque.resize(3);
    m_arm_ft_force.resize(3);
    m_arm_pose.resize(7);
    m_arm_tool = 0;

    const int qsize = 3;
    m_armJointAnglesSubscriber = make_unique<ros::Subscriber>
      (m_genericNodeHandle ->
       subscribe("/owlat_sim/ARM_JOINT_ANGLES", qsize,
       &OwlatInterface::armJointAnglesCallback, this));

    m_armJointAccelerationsSubscriber = make_unique<ros::Subscriber>
      (m_genericNodeHandle ->
       subscribe("/owlat_sim/ARM_JOINT_ACCELERATIONS", qsize,
       &OwlatInterface::armJointAccelerationsCallback, this));

    m_armJointTorquesSubscriber = make_unique<ros::Subscriber>
      (m_genericNodeHandle ->
       subscribe("/owlat_sim/ARM_JOINT_TORQUES", qsize,
       &OwlatInterface::armJointTorquesCallback, this));

    m_armJointVelocitiesSubscriber = make_unique<ros::Subscriber>
      (m_genericNodeHandle ->
       subscribe("/owlat_sim/ARM_JOINT_VELOCITIES", qsize,
       &OwlatInterface::armJointAnglesCallback, this));

    m_armFTTorqueSubscriber = make_unique<ros::Subscriber>
      (m_genericNodeHandle ->
       subscribe("/owlat_sim/ARM_FT_TORQUE", qsize,
       &OwlatInterface::armFTTorqueCallback, this));

   m_armFTForceSubscriber = make_unique<ros::Subscriber>
        (m_genericNodeHandle ->
         subscribe("/owlat_sim/ARM_FT_FORCE", qsize,
         &OwlatInterface::armFTForceCallback, this));

   m_armPoseSubscriber = make_unique<ros::Subscriber>
        (m_genericNodeHandle ->
         subscribe("/owlat_sim/ARM_POSE", qsize,
         &OwlatInterface::armPoseCallback, this));

   m_armToolSubscriber = make_unique<ros::Subscriber>
        (m_genericNodeHandle ->
         subscribe("/owlat_sim/ARM_TOOL", qsize,
         &OwlatInterface::armToolCallback, this));

    // Initialize pointers
    m_owlatUnstowClient =
      make_unique<OwlatUnstowActionClient>(Name_OwlatUnstow, true);
    m_owlatStowClient =
      make_unique<OwlatStowActionClient>(Name_OwlatStow, true);
    m_owlatArmMoveCartesianClient =
      make_unique<OwlatArmMoveCartesianActionClient>(Name_OwlatArmMoveCartesian,
                                                     true);
    m_owlatArmMoveCartesianGuardedClient =
      make_unique<OwlatArmMoveCartesianGuardedActionClient>
      (Name_OwlatArmMoveCartesianGuarded, true);
    m_owlatArmMoveJointClient =
      make_unique<OwlatArmMoveJointActionClient>(Name_OwlatArmMoveJoint, true);
    m_owlatArmMoveJointsClient =
      make_unique<OwlatArmMoveJointsActionClient>(Name_OwlatArmMoveJoints, true);
    m_owlatArmMoveJointsGuardedClient =
      make_unique<OwlatArmMoveJointsGuardedActionClient>
      (Name_OwlatArmMoveJointsGuarded, true);
    m_owlatArmPlaceToolClient =
      make_unique<OwlatArmPlaceToolActionClient>(Name_OwlatArmPlaceTool, true);
    m_owlatArmSetToolClient =
      make_unique<OwlatArmSetToolActionClient>(Name_OwlatArmSetTool, true);
    m_owlatArmStopClient =
      make_unique<OwlatArmStopActionClient>(Name_OwlatArmStop, true);
    m_owlatArmTareFSClient =
      make_unique<OwlatArmTareFSActionClient>(Name_OwlatArmTareFS, true);
    m_owlatTaskPSPClient =
      make_unique<OwlatTaskPSPActionClient>(Name_OwlatTaskPSP, true);
    m_owlatTaskScoopClient =
      make_unique<OwlatTaskScoopActionClient>(Name_OwlatTaskScoop, true);

    // Connect to action servers
    if (! m_owlatUnstowClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("OWLAT UNSTOW action server did not connect!");
    }
    if (! m_owlatStowClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("OWLAT STOW action server did not connect!");
    }
    if (! m_owlatArmMoveCartesianClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("OWLAT ARM_MOVE_CARTESIAN action server did not connect!");
    }
    if (! m_owlatArmMoveCartesianGuardedClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("OWLAT ARM_MOVE_CARTESIAN_GUARDED action server did not connect!");
    }
    if (! m_owlatArmMoveJointClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("OWLAT ARM_MOVE_JOINT action server did not connect!");
    }
    if (! m_owlatArmMoveJointsClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("OWLAT ARM_MOVE_JOINTS action server did not connect!");
    }
    if (! m_owlatArmMoveJointsGuardedClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("OWLAT ARM_MOVE_JOINTS_GUARDED action server did not connect!");
    }
    if (! m_owlatArmPlaceToolClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("OWLAT ARM_PLACE_TOOL action server did not connect!");
    }
    if (! m_owlatArmSetToolClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("OWLAT ARM_SET_TOOL action server did not connect!");
    }
    if (! m_owlatArmStopClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("OWLAT ARM_STOP action server did not connect!");
    }
    if (! m_owlatArmTareFSClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("OWLAT ARM_TARE_FS action server did not connect!");
    }
    if (! m_owlatTaskPSPClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("OWLAT TASK_PSP action server did not connect!");
    }
    if (! m_owlatTaskScoopClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("OWLAT TASK_SCOOP action server did not connect!");
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
                                            const vector<double>& position,
                                            const vector<double>& orientation,
                                            int id)
{
  if (! markOperationRunning (Name_OwlatArmMoveCartesian, id)) return;
  thread action_thread (&OwlatInterface::owlatArmMoveCartesianAction, this,
                        frame, relative, position, orientation, id);
  action_thread.detach();
}

void OwlatInterface::owlatArmMoveCartesianAction (int frame, bool relative,
                                                  const vector<double>& position,
                                                  const vector<double>& orientation,
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

void OwlatInterface::owlatArmMoveCartesianGuarded (int frame, bool relative,
                                                   const vector<double>& position,
                                                   const vector<double>& orientation,
                                                   bool retracting,
                                                   double force_threshold,
                                                   double torque_threshold,int id)
{
  if (! markOperationRunning (Name_OwlatArmMoveCartesianGuarded, id)) return;
  thread action_thread (&OwlatInterface::owlatArmMoveCartesianGuardedAction,
                        this, frame, relative, position, orientation,
                        retracting, force_threshold, torque_threshold, id);
  action_thread.detach();
}

void OwlatInterface::owlatArmMoveCartesianGuardedAction (int frame, bool relative,
                                                         const vector<double>& position,
                                                         const vector<double>& orientation,
                                                         bool retracting,
                                                         double force_threshold,
                                                         double torque_threshold, int id)
{
  ARM_MOVE_CARTESIAN_GUARDEDGoal goal;
  goal.frame.value = frame;
  goal.relative = relative;
  goal.pose.position.x = position[0];
  goal.pose.position.y = position[1];
  goal.pose.position.z = position[2];
  goal.pose.orientation.x = orientation[0];
  goal.pose.orientation.y = orientation[1];
  goal.pose.orientation.z = orientation[2];
  goal.pose.orientation.w = orientation[3];
  goal.retracting = retracting;
  goal.force_threshold = force_threshold;
  goal.torque_threshold = torque_threshold;
  string opname = Name_OwlatArmMoveCartesianGuarded;  // shorter version

  runAction<actionlib::SimpleActionClient<ARM_MOVE_CARTESIAN_GUARDEDAction>,
            ARM_MOVE_CARTESIAN_GUARDEDGoal,
            ARM_MOVE_CARTESIAN_GUARDEDResultConstPtr,
            ARM_MOVE_CARTESIAN_GUARDEDFeedbackConstPtr>
    (opname, m_owlatArmMoveCartesianGuardedClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ARM_MOVE_CARTESIAN_GUARDEDFeedbackConstPtr> (opname),
     default_action_done_cb<ARM_MOVE_CARTESIAN_GUARDEDResultConstPtr> (opname));
}

void OwlatInterface::owlatArmMoveJoint (bool relative,
                                        int joint, double angle,
                                        int id)
{
  if (! markOperationRunning (Name_OwlatArmMoveJoint, id)) return;
  thread action_thread (&OwlatInterface::owlatArmMoveJointAction,
                        this, relative, joint, angle, id);
  action_thread.detach();
}

void OwlatInterface::owlatArmMoveJointAction (bool relative,
                                              int joint, double angle,
                                              int id)
{
  ARM_MOVE_JOINTGoal goal;
  goal.relative = relative;
  goal.joint = joint;
  goal.angle = angle;
  string opname = Name_OwlatArmMoveJoint;  // shorter version

  runAction<actionlib::SimpleActionClient<ARM_MOVE_JOINTAction>,
            ARM_MOVE_JOINTGoal,
            ARM_MOVE_JOINTResultConstPtr,
            ARM_MOVE_JOINTFeedbackConstPtr>
    (opname, m_owlatArmMoveJointClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ARM_MOVE_JOINTFeedbackConstPtr> (opname),
     default_action_done_cb<ARM_MOVE_JOINTResultConstPtr> (opname));
}

void OwlatInterface::owlatArmMoveJoints (bool relative,
                                         const vector<double>& angles,
                                         int id)
{
  if (! markOperationRunning (Name_OwlatArmMoveJoints, id)) return;
  thread action_thread (&OwlatInterface::owlatArmMoveJointsAction,
                        this, relative, angles, id);
  action_thread.detach();
}

void OwlatInterface::owlatArmMoveJointsAction (bool relative,
                                               const vector<double>& angles,
                                               int id)
{

  ARM_MOVE_JOINTSGoal goal;
  goal.relative = relative;
  for(int i = 0; i < goal.angles.size(); i++){
    goal.angles[i] = angles[i];
  }
  string opname = Name_OwlatArmMoveJoints;  // shorter version

  runAction<actionlib::SimpleActionClient<ARM_MOVE_JOINTSAction>,
            ARM_MOVE_JOINTSGoal,
            ARM_MOVE_JOINTSResultConstPtr,
            ARM_MOVE_JOINTSFeedbackConstPtr>
    (opname, m_owlatArmMoveJointsClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ARM_MOVE_JOINTSFeedbackConstPtr> (opname),
     default_action_done_cb<ARM_MOVE_JOINTSResultConstPtr> (opname));
}

void OwlatInterface::owlatArmMoveJointsGuarded (bool relative,
                                                const vector<double>& angles,
                                                bool retracting,
                                                double force_threshold,
                                                double torque_threshold,
                                                int id)
{
  if (! markOperationRunning (Name_OwlatArmMoveJointsGuarded, id)) return;
  thread action_thread (&OwlatInterface::owlatArmMoveJointsGuardedAction,
                        this, relative, angles, retracting, force_threshold,
                        torque_threshold, id);
  action_thread.detach();
}

void OwlatInterface::owlatArmMoveJointsGuardedAction (bool relative,
                                                      const vector<double>& angles,
                                                      bool retracting,
                                                      double force_threshold,
                                                      double torque_threshold,
                                                      int id)
{

  ARM_MOVE_JOINTS_GUARDEDGoal goal;
  goal.relative = relative;
  for(int i = 0; i < goal.angles.size(); i++){
    goal.angles[i] = angles[i];
  }
  goal.retracting = retracting;
  goal.force_threshold = force_threshold;
  goal.torque_threshold = torque_threshold;
  string opname = Name_OwlatArmMoveJointsGuarded;  // shorter version

  runAction<actionlib::SimpleActionClient<ARM_MOVE_JOINTS_GUARDEDAction>,
            ARM_MOVE_JOINTS_GUARDEDGoal,
            ARM_MOVE_JOINTS_GUARDEDResultConstPtr,
            ARM_MOVE_JOINTS_GUARDEDFeedbackConstPtr>
    (opname, m_owlatArmMoveJointsGuardedClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ARM_MOVE_JOINTS_GUARDEDFeedbackConstPtr> (opname),
     default_action_done_cb<ARM_MOVE_JOINTS_GUARDEDResultConstPtr> (opname));
}

void OwlatInterface::owlatArmPlaceTool (int frame, bool relative,
                                        const vector<double>& position,
                                        const vector<double>& normal,
                                        double distance, double overdrive,
                                        bool retracting, double force_threshold,
                                        double torque_threshold, int id)
{
  if (! markOperationRunning (Name_OwlatArmPlaceTool, id)) return;
  thread action_thread (&OwlatInterface::owlatArmPlaceToolAction,
                        this, frame, relative, position, normal,
                        distance, overdrive, retracting, force_threshold,
                        torque_threshold, id);
  action_thread.detach();
}

void OwlatInterface::owlatArmPlaceToolAction (int frame, bool relative,
                                              const vector<double>& position,
                                              const vector<double>& normal,
                                              double distance, double overdrive,
                                              bool retracting, double force_threshold,
                                              double torque_threshold, int id)
{
  ARM_PLACE_TOOLGoal goal;
  goal.frame.value = frame;
  goal.relative = relative;
  for(int i = 0; i < goal.position.size(); i++){
    goal.position[i] = position[i];
    goal.normal[i] = normal[i];
  }
  goal.distance = distance;
  goal.overdrive = overdrive;
  goal.retracting = retracting;
  goal.force_threshold = force_threshold;
  goal.torque_threshold = torque_threshold;
  string opname = Name_OwlatArmPlaceTool;  // shorter version

  runAction<actionlib::SimpleActionClient<ARM_PLACE_TOOLAction>,
            ARM_PLACE_TOOLGoal,
            ARM_PLACE_TOOLResultConstPtr,
            ARM_PLACE_TOOLFeedbackConstPtr>
    (opname, m_owlatArmPlaceToolClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ARM_PLACE_TOOLFeedbackConstPtr> (opname),
     default_action_done_cb<ARM_PLACE_TOOLResultConstPtr> (opname));
}

void OwlatInterface::owlatArmSetTool (int tool, int id)
{
  if (! markOperationRunning (Name_OwlatArmSetTool, id)) return;
  thread action_thread (&OwlatInterface::owlatArmSetToolAction, this, tool, id);
  action_thread.detach();
}

void OwlatInterface::owlatArmSetToolAction (int tool, int id)
{
  ARM_SET_TOOLGoal goal;
  goal.tool.value = tool;
  string opname = Name_OwlatArmSetTool;  // shorter version

  runAction<actionlib::SimpleActionClient<ARM_SET_TOOLAction>,
            ARM_SET_TOOLGoal,
            ARM_SET_TOOLResultConstPtr,
            ARM_SET_TOOLFeedbackConstPtr>
    (opname, m_owlatArmSetToolClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ARM_SET_TOOLFeedbackConstPtr> (opname),
     default_action_done_cb<ARM_SET_TOOLResultConstPtr> (opname));
}

void OwlatInterface::owlatArmStop (int id)
{
  if (! markOperationRunning (Name_OwlatArmStop, id)) return;
  thread action_thread (&OwlatInterface::owlatArmStopAction, this, id);
  action_thread.detach();
}

void OwlatInterface::owlatArmStopAction (int id)
{
  ARM_STOPGoal goal;
  string opname = Name_OwlatArmStop;  // shorter version

  runAction<actionlib::SimpleActionClient<ARM_STOPAction>,
            ARM_STOPGoal,
            ARM_STOPResultConstPtr,
            ARM_STOPFeedbackConstPtr>
    (opname, m_owlatArmStopClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ARM_STOPFeedbackConstPtr> (opname),
     default_action_done_cb<ARM_STOPResultConstPtr> (opname));
}

void OwlatInterface::owlatArmTareFS (int id)
{
  if (! markOperationRunning (Name_OwlatArmTareFS, id)) return;
  thread action_thread (&OwlatInterface::owlatArmTareFSAction, this, id);
  action_thread.detach();
}

void OwlatInterface::owlatArmTareFSAction (int id)
{
  ARM_TARE_FSGoal goal;
  string opname = Name_OwlatArmTareFS;  // shorter version

  runAction<actionlib::SimpleActionClient<ARM_TARE_FSAction>,
            ARM_TARE_FSGoal,
            ARM_TARE_FSResultConstPtr,
            ARM_TARE_FSFeedbackConstPtr>
    (opname, m_owlatArmTareFSClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ARM_TARE_FSFeedbackConstPtr> (opname),
     default_action_done_cb<ARM_TARE_FSResultConstPtr> (opname));
}

void OwlatInterface::owlatTaskPSP (int frame, bool relative,
                                   const vector<double>& point,
                                   const vector<double>& normal,
                                   double max_depth,
                                   double max_force, int id)
{
  if (! markOperationRunning (Name_OwlatTaskPSP, id)) return;
  thread action_thread (&OwlatInterface::owlatTaskPSPAction,
                        this, frame, relative, point, normal,
                        max_depth, max_force, id);
  action_thread.detach();
}

void OwlatInterface::owlatTaskPSPAction (int frame, bool relative,
                                         const vector<double>& point,
                                         const vector<double>& normal,
                                         double max_depth,
                                         double max_force, int id)
{
  TASK_PSPGoal goal;
  goal.frame.value = frame;
  goal.relative = relative;
  for(int i = 0; i < goal.point.size(); i++){
    goal.point[i] = point[i];
    goal.normal[i] = normal[i];
  }
  goal.max_depth = max_depth;
  goal.max_force = max_force;
  string opname = Name_OwlatTaskPSP;  // shorter version

  runAction<actionlib::SimpleActionClient<TASK_PSPAction>,
            TASK_PSPGoal,
            TASK_PSPResultConstPtr,
            TASK_PSPFeedbackConstPtr>
    (opname, m_owlatTaskPSPClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<TASK_PSPFeedbackConstPtr> (opname),
     task_psp_done_cb<static_cast<int>(LanderOps::OwlatArmTaskPSP),
     TASK_PSPResultConstPtr>);
}

void OwlatInterface::owlatTaskScoop (int frame, bool relative,
                                     const vector<double>& point,
                                     const vector<double>& normal, int id)
{
  if (! markOperationRunning (Name_OwlatTaskPSP, id)) return;
  thread action_thread (&OwlatInterface::owlatTaskScoopAction,
                        this, frame, relative, point, normal,
                        id);
  action_thread.detach();
}

void OwlatInterface::owlatTaskScoopAction (int frame, bool relative,
                                           const vector<double>& point,
                                           const vector<double>& normal, int id)
{
  TASK_SCOOPGoal goal;
  goal.frame.value = frame;
  goal.relative = relative;
  for(int i = 0; i < goal.point.size(); i++){
    goal.point[i] = point[i];
    goal.normal[i] = normal[i];
  }
  string opname = Name_OwlatTaskScoop;  // shorter version

  runAction<actionlib::SimpleActionClient<TASK_SCOOPAction>,
            TASK_SCOOPGoal,
            TASK_SCOOPResultConstPtr,
            TASK_SCOOPFeedbackConstPtr>
    (opname, m_owlatTaskScoopClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<TASK_SCOOPFeedbackConstPtr> (opname),
     default_action_done_cb<TASK_SCOOPResultConstPtr> (opname));
}

void OwlatInterface::armJointAnglesCallback
(const owlat_sim_msgs::ARM_JOINT_ANGLES::ConstPtr& msg)
{
  copy(msg->value.begin(), msg->value.end(), m_arm_joint_angles.begin());
  publish("ArmJointAngles", m_arm_joint_angles);
}

void OwlatInterface::armJointAccelerationsCallback
(const owlat_sim_msgs::ARM_JOINT_ACCELERATIONS::ConstPtr& msg)
{
  copy(msg->value.begin(), msg->value.end(),
            m_arm_joint_accelerations.begin());
  publish("ArmJointAccelerations", m_arm_joint_accelerations);
}

void OwlatInterface::armJointTorquesCallback
(const owlat_sim_msgs::ARM_JOINT_TORQUES::ConstPtr& msg)
{
  copy(msg->value.begin(), msg->value.end(), m_arm_joint_torques.begin());
  publish("ArmJointTorques", m_arm_joint_torques);
}

void OwlatInterface::armJointVelocitiesCallback
(const owlat_sim_msgs::ARM_JOINT_VELOCITIES::ConstPtr& msg)
{
  copy(msg->value.begin(), msg->value.end(), m_arm_joint_velocities.begin());
  publish("ArmJointVelocities", m_arm_joint_velocities);
}

void OwlatInterface::armFTTorqueCallback
(const owlat_sim_msgs::ARM_FT_TORQUE::ConstPtr& msg)
{
  copy(msg->value.begin(), msg->value.end(), m_arm_ft_torque.begin());
  publish("ArmFTTorque", m_arm_ft_torque);
}

void OwlatInterface::armFTForceCallback
(const owlat_sim_msgs::ARM_FT_FORCE::ConstPtr& msg)
{
  copy(msg->value.begin(), msg->value.end(), m_arm_ft_force.begin());
  publish("ArmFTForce", m_arm_ft_force);
}

void OwlatInterface::armPoseCallback
(const owlat_sim_msgs::ARM_POSE::ConstPtr& msg)
{
  m_arm_pose[0] = msg->value.position.x;
  m_arm_pose[1] = msg->value.position.y;
  m_arm_pose[2] = msg->value.position.z;
  m_arm_pose[3] = msg->value.orientation.x;
  m_arm_pose[4] = msg->value.orientation.y;
  m_arm_pose[5] = msg->value.orientation.z;
  m_arm_pose[6] = msg->value.orientation.w;
  publish("ArmPose", m_arm_pose);
}

void OwlatInterface::armToolCallback
(const owlat_sim_msgs::ARM_TOOL::ConstPtr& msg)
{
  m_arm_tool = msg->value.value;
  publish("ArmTool", m_arm_tool);
}

Value OwlatInterface::getArmJointAngles()
{
  return(Value(m_arm_joint_angles));
}

Value OwlatInterface::getArmJointAccelerations()
{
  return(Value(m_arm_joint_accelerations));
}

Value OwlatInterface::getArmJointTorques()
{
  return(Value(m_arm_joint_torques));
}

Value OwlatInterface::getArmJointVelocities()
{
  return(Value(m_arm_joint_velocities));
}

Value OwlatInterface::getArmFTTorque()
{
  return(Value(m_arm_ft_torque));
}

Value OwlatInterface::getArmFTForce()
{
  return(Value(m_arm_ft_force));
}

Value OwlatInterface::getArmPose()
{
  return(Value(m_arm_pose));
}

Value OwlatInterface::getArmTool()
{
  return(Value(m_arm_tool));
}

Value OwlatInterface::getPSPStopReason()
{
  return(Value(PSPStopReasonVar));
}

