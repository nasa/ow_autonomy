// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
// Exploration Research and Simulation can be found in README.md in
// the root directory of this repository.

#include <thread>
#include <vector>
#include "OwlatInterface.h"
#include <ArrayImpl.hh>
#include <functional>
#include "subscriber.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using std::hash;
using std::copy;
using std::string;
using std::thread;
using std::string;
using std::vector;
using std::make_unique;
using namespace owlat_sim_msgs;
using namespace owl_msgs;
using namespace PLEXIL;

const string Name_ArmFindSurface = "ArmFindSurface";
const string Name_Tare = "ArmTareFTSensor";
const string Name_SetTool =       "ArmSetTool";
const string Name_TaskPSP =          "TaskPSP";
const string Name_TaskShearBevameter =          "TaskShearBevameter";
const string Name_TaskPenetrometer =          "TaskPenetrometer";
const string Name_TaskScoopCircular =        "TaskScoopCircular";
const string Name_TaskScoopLinear =        "TaskScoopLinear";

const string Name_OwlatArmMoveJoints =    "/owlat_sim/ARM_MOVE_JOINTS";
const string Name_OwlatArmMoveJointsGuarded =
  "/owlat_sim/ARM_MOVE_JOINTS_GUARDED";
const string Name_OwlatArmPlaceTool =     "/owlat_sim/ARM_PLACE_TOOL";
const string Name_Discard = "TaskDiscardSample";

const size_t OwlatJoints = 6;

static vector<string> LanderOpNames = {
  Name_ArmFindSurface,
  Name_Discard,
  Name_OwlatArmMoveJoints,
  Name_OwlatArmMoveJointsGuarded,
  Name_OwlatArmPlaceTool,
  Name_SetTool,
  Name_Tare,
  Name_TaskPSP,
  Name_TaskShearBevameter,
  Name_TaskPenetrometer,
  Name_TaskScoopCircular,
  Name_TaskScoopLinear
};

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
    LanderInterface::initialize();

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
    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/owlat_sim/ARM_JOINT_ANGLES", qsize,
                  &OwlatInterface::armJointAnglesCallback, this)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/owlat_sim/ARM_JOINT_ACCELERATIONS", qsize,
                  &OwlatInterface::armJointAccelerationsCallback, this)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/owlat_sim/ARM_JOINT_TORQUES", qsize,
                  &OwlatInterface::armJointTorquesCallback, this)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/owlat_sim/ARM_JOINT_VELOCITIES", qsize,
                  &OwlatInterface::armJointVelocitiesCallback, this)));


    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/owlat_sim/ARM_FT_TORQUE", qsize,
                  &OwlatInterface::armFTTorqueCallback, this)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/owlat_sim/ARM_FT_FORCE", qsize,
                  &OwlatInterface::armFTForceCallback, this)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/owlat_sim/ARM_POSE", qsize,
                  &OwlatInterface::armPoseCallback, this)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/owlat_sim/ARM_TOOL", qsize,
                  &OwlatInterface::armToolCallback, this)));

    m_subscribers.push_back
      (make_unique<ros::Subscriber>
       (m_genericNodeHandle ->
        subscribe("/pan_tilt_position", qsize,
                  &OwlatInterface::panTiltCallback, this)));

    // Initialize pointers
    m_armFindSurfaceClient =
      make_unique<ArmFindSurfaceActionClient>(Name_ArmFindSurface, true);
    m_taskDiscardSampleClient =
      make_unique<TaskDiscardSampleActionClient>(Name_Discard, true);
    m_owlatArmMoveJointsClient =
      make_unique<OwlatArmMoveJointsActionClient>(Name_OwlatArmMoveJoints, true);
    m_owlatArmMoveJointsGuardedClient =
      make_unique<OwlatArmMoveJointsGuardedActionClient>
      (Name_OwlatArmMoveJointsGuarded, true);
    m_owlatArmPlaceToolClient =
      make_unique<OwlatArmPlaceToolActionClient>(Name_OwlatArmPlaceTool, true);
    m_armSetToolClient =
      make_unique<ArmSetToolActionClient>(Name_SetTool, true);
    m_armTareFTSensorClient =
      make_unique<ArmTareFTSensorActionClient>(Name_Tare, true);
    m_taskPSPClient =
      make_unique<TaskPSPActionClient>(Name_TaskPSP, true);
    m_taskShearBevameterClient =
      make_unique<TaskShearBevameterActionClient>(Name_TaskShearBevameter, true);
    m_taskPenetrometerClient =
      make_unique<TaskPenetrometerActionClient>(Name_TaskPenetrometer, true);
    m_taskScoopCircularClient =
      make_unique<TaskScoopCircularActionClient>(Name_TaskScoopCircular, true);
    m_taskScoopLinearClient =
      make_unique<TaskScoopLinearActionClient>(Name_TaskScoopLinear, true);

    // Connect to action servers
    connectActionServer (m_armFindSurfaceClient, Name_ArmFindSurface,
                         "/ArmFindSurface/status");
    connectActionServer (m_taskDiscardSampleClient, Name_Discard,
                         "/TaskDiscardSample/status");

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
    if (! m_armSetToolClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("OWLAT ArmSetTool action server did not connect!");
    }
    if (! m_armTareFTSensorClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("ArmTareFTSensor action server did not connect!");
    }
    if (! m_taskShearBevameterClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("TaskShearBevameter action server did not connect!");
    }
    if (! m_taskPenetrometerClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("TaskPenetrometer action server did not connect!");
    }
    if (! m_taskScoopCircularClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("TaskScoopCircular action server did not connect!");
    }
    if (! m_taskScoopLinearClient->waitForServer
        (ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("TaskScoopLinear action server did not connect!");
    }
  }
}


/////////////////////////////// OWLAT Interface ////////////////////////////////


void OwlatInterface::armFindSurface (int frame,
                                     bool relative,
                                     const std::vector<double>& pos,
                                     const std::vector<double>& normal,
                                     double distance,
                                     double overdrive,
                                     double force_threshold,
                                     double torque_threshold,
                                     int id)
{
  if (! markOperationRunning (Name_ArmFindSurface, id)) return;

  thread action_thread (&OwlatInterface::armFindSurfaceAction, this,
			frame, relative, pos, normal, distance, overdrive,
                        force_threshold, torque_threshold,
                        id);
  action_thread.detach();
}

void OwlatInterface::armFindSurfaceAction (int frame, bool relative,
                                           const std::vector<double>& pos,
                                           const std::vector<double>& normal,
                                           double distance, double overdrive,
                                           double force_threshold, double torque_threshold,
                                           int id)
{
  ArmFindSurfaceGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  goal.position[0] = pos[0];
  goal.position[1] = pos[1];
  goal.position[2] = pos[2];
  goal.normal[0] = normal[0];
  goal.normal[1] = normal[1];
  goal.normal[2] = normal[2];
  goal.distance = distance;
  goal.overdrive = overdrive;
  goal.force_threshold = force_threshold;
  goal.torque_threshold = torque_threshold;

  // Fill this out.
  ROS_INFO ("Starting ArmFindSurface (frame=%d, relative=%d)", frame, relative);

  runAction<actionlib::SimpleActionClient<ArmFindSurfaceAction>,
            ArmFindSurfaceGoal,
            ArmFindSurfaceResultConstPtr,
            ArmFindSurfaceFeedbackConstPtr>
	    (Name_ArmFindSurface, m_armFindSurfaceClient, goal, id,
	     default_action_active_cb (Name_ArmFindSurface),
	     default_action_feedback_cb<ArmFindSurfaceFeedbackConstPtr>
	     (Name_ArmFindSurface),
	     default_action_done_cb<ArmFindSurfaceResultConstPtr>
	     (Name_ArmFindSurface));
}


void OwlatInterface::taskDiscardSample (int frame, bool relative,
                                        const std::vector<double>& point,
                                        double height, int id)
{
  if (! markOperationRunning (Name_Discard, id)) return;
  thread action_thread (&OwlatInterface::taskDiscardSampleAction, this,
                        frame, relative, point, height, id);
  action_thread.detach();
}

void OwlatInterface::taskDiscardSampleAction (int frame, bool relative,
                                              const std::vector<double>& point,
                                              double height, int id)
{
  TaskDiscardSampleGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  for (size_t i = 0; i < 3; i++) {
    goal.point[i] = point[i];
  }
  goal.height = height;

  ROS_INFO ("Starting TaskDiscardSample(x=%.2f, y=%.2f, z=%.2f)",
            point[0], point[1], point[2]);

  runAction<actionlib::SimpleActionClient<TaskDiscardSampleAction>,
            TaskDiscardSampleGoal,
            TaskDiscardSampleResultConstPtr,
            TaskDiscardSampleFeedbackConstPtr>
    (Name_Discard, m_taskDiscardSampleClient, goal, id,
     default_action_active_cb (Name_Discard),
     default_action_feedback_cb<TaskDiscardSampleFeedbackConstPtr> (Name_Discard),
     default_action_done_cb<TaskDiscardSampleResultConstPtr> (Name_Discard));
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
  string opname = Name_OwlatArmMoveJoints;

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
  string opname = Name_OwlatArmMoveJointsGuarded;

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
  string opname = Name_OwlatArmPlaceTool;

  runAction<actionlib::SimpleActionClient<ARM_PLACE_TOOLAction>,
            ARM_PLACE_TOOLGoal,
            ARM_PLACE_TOOLResultConstPtr,
            ARM_PLACE_TOOLFeedbackConstPtr>
    (opname, m_owlatArmPlaceToolClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ARM_PLACE_TOOLFeedbackConstPtr> (opname),
     default_action_done_cb<ARM_PLACE_TOOLResultConstPtr> (opname));
}

void OwlatInterface::armSetTool (int tool, int id)
{
  if (! markOperationRunning (Name_SetTool, id)) return;
  thread action_thread (&OwlatInterface::armSetToolAction, this, tool, id);
  action_thread.detach();
}

void OwlatInterface::armSetToolAction (int tool, int id)
{
  ArmSetToolGoal goal;
  goal.tool = tool;
  string opname = Name_SetTool;

  runAction<actionlib::SimpleActionClient<ArmSetToolAction>,
            ArmSetToolGoal,
            ArmSetToolResultConstPtr,
            ArmSetToolFeedbackConstPtr>
    (opname, m_armSetToolClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ArmSetToolFeedbackConstPtr> (opname),
     default_action_done_cb<ArmSetToolResultConstPtr> (opname));
}

void OwlatInterface::armTareFTSensor (int id)
{
  if (! markOperationRunning (Name_Tare, id)) return;
  thread action_thread (&OwlatInterface::nullaryAction<
                        ArmTareFTSensorActionClient,
                        ArmTareFTSensorGoal,
                        ArmTareFTSensorResultConstPtr,
                        ArmTareFTSensorFeedbackConstPtr>,
                        this, id, Name_Tare, std::ref(m_armTareFTSensorClient));
  action_thread.detach();
}

void OwlatInterface::taskShearBevameter (int frame,
                                         bool relative,
                                         const std::vector<double>& point,
                                         const std::vector<double>& normal,
                                         double preload,
                                         double max_torque,
                                         int id)
{
  if (! markOperationRunning (Name_TaskShearBevameter, id)) return;
  thread action_thread (&OwlatInterface::taskShearBevameterAction,
                        this, frame, relative, point, normal,
                        preload, max_torque, id);
  action_thread.detach();
}


void OwlatInterface::taskPSP (int frame,
                              bool relative,
                              const vector<double>& point,
                              const vector<double>& normal,
                              double max_depth,
                              double max_force,
                              int id)
{
  if (! markOperationRunning (Name_TaskPSP, id)) return;
  thread action_thread (&OwlatInterface::taskPSPAction,
                        this, frame, relative, point, normal,
                        max_depth, max_force, id);
  action_thread.detach();
}

void OwlatInterface::taskPenetrometer (int frame,
                                       bool relative,
                                       const vector<double>& point,
                                       const vector<double>& normal,
                                       double max_depth,
                                       double max_force,
                                       int id)
{
  if (! markOperationRunning (Name_TaskPenetrometer, id)) return;
  thread action_thread (&OwlatInterface::taskPenetrometerAction,
                        this, frame, relative, point, normal,
                        max_depth, max_force, id);
  action_thread.detach();
}

void OwlatInterface::taskPSPAction (int frame,
                                    bool relative,
				    const vector<double>& point,
				    const vector<double>& normal,
				    double max_depth,
				    double max_force,
                                    int id)
{
  TaskPSPGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  for(int i = 0; i < goal.point.size(); i++){
    goal.point[i] = point[i];
    goal.normal[i] = normal[i];
  }
  goal.max_depth = max_depth;
  goal.max_force = max_force;
  string opname = Name_TaskPSP;

  runAction<actionlib::SimpleActionClient<TaskPSPAction>,
            TaskPSPGoal,
            TaskPSPResultConstPtr,
            TaskPSPFeedbackConstPtr>
    (opname, m_taskPSPClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<TaskPSPFeedbackConstPtr> (opname),
     default_action_done_cb<TaskPSPResultConstPtr> (opname));
}

void OwlatInterface::taskShearBevameterAction (int frame,
                                               bool relative,
                                               const std::vector<double>& point,
                                               const std::vector<double>& normal,
                                               double preload,
                                               double max_torque,
                                               int id)
{
  TaskShearBevameterGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  for(int i = 0; i < goal.point.size(); i++){
    goal.point[i] = point[i];
    goal.normal[i] = normal[i];
  }
  goal.preload = preload;
  goal.max_torque = max_torque;
  string opname = Name_TaskShearBevameter;

  runAction<actionlib::SimpleActionClient<TaskShearBevameterAction>,
            TaskShearBevameterGoal,
            TaskShearBevameterResultConstPtr,
            TaskShearBevameterFeedbackConstPtr>
    (opname, m_taskShearBevameterClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<TaskShearBevameterFeedbackConstPtr> (opname),
     default_action_done_cb<TaskShearBevameterResultConstPtr> (opname));
}


void OwlatInterface::taskPenetrometerAction (int frame,
                                             bool relative,
                                             const vector<double>& point,
                                             const vector<double>& normal,
                                             double max_depth,
                                             double max_force,
                                             int id)
{
  TaskPenetrometerGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  for(int i = 0; i < goal.point.size(); i++){
    goal.point[i] = point[i];
    goal.normal[i] = normal[i];
  }
  goal.max_depth = max_depth;
  goal.max_force = max_force;
  string opname = Name_TaskPenetrometer;

  runAction<actionlib::SimpleActionClient<TaskPenetrometerAction>,
            TaskPenetrometerGoal,
            TaskPenetrometerResultConstPtr,
            TaskPenetrometerFeedbackConstPtr>
    (opname, m_taskPenetrometerClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<TaskPenetrometerFeedbackConstPtr> (opname),
     default_action_done_cb<TaskPenetrometerResultConstPtr> (opname));
}

void OwlatInterface::taskScoopCircular (int frame,
                                        bool relative,
                                        const vector<double>& point,
                                        const vector<double>& normal,
                                        double depth,
                                        double scoop_angle,
                                        int id)
{
  if (! markOperationRunning (Name_TaskScoopCircular, id)) return;
  thread action_thread (&OwlatInterface::taskScoopCircularAction,
                        this, frame, relative, point, normal, depth, scoop_angle,
                        id);
  action_thread.detach();
}

void OwlatInterface::taskScoopCircularAction (int frame, bool relative,
                                              const vector<double>& point,
                                              const vector<double>& normal,
                                              double depth, double scoop_angle,
                                              int id)
{
  TaskScoopCircularGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  for(int i = 0; i < goal.point.size(); i++){
    goal.point[i] = point[i];
    goal.normal[i] = normal[i];
  }
  goal.depth = depth;
  goal.scoop_angle = scoop_angle;
  string opname = Name_TaskScoopCircular;

  runAction<actionlib::SimpleActionClient<TaskScoopCircularAction>,
            TaskScoopCircularGoal,
            TaskScoopCircularResultConstPtr,
            TaskScoopCircularFeedbackConstPtr>
    (opname, m_taskScoopCircularClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<TaskScoopCircularFeedbackConstPtr> (opname),
     default_action_done_cb<TaskScoopCircularResultConstPtr> (opname));
}

void OwlatInterface::taskScoopLinear (int frame,
                                        bool relative,
                                        const vector<double>& point,
                                        const vector<double>& normal,
                                        double depth,
                                        double length,
                                        int id)
{
  if (! markOperationRunning (Name_TaskScoopLinear, id)) return;
  thread action_thread (&OwlatInterface::taskScoopLinearAction,
                        this, frame, relative, point, normal, depth, length,
                        id);
  action_thread.detach();
}

void OwlatInterface::taskScoopLinearAction (int frame, bool relative,
                                              const vector<double>& point,
                                              const vector<double>& normal,
                                              double depth, double length,
                                              int id)
{
  TaskScoopLinearGoal goal;
  goal.frame = frame;
  goal.relative = relative;
  for(int i = 0; i < goal.point.size(); i++){
    goal.point[i] = point[i];
    goal.normal[i] = normal[i];
  }
  goal.depth = depth;
  goal.length = length;
  string opname = Name_TaskScoopLinear;

  runAction<actionlib::SimpleActionClient<TaskScoopLinearAction>,
            TaskScoopLinearGoal,
            TaskScoopLinearResultConstPtr,
            TaskScoopLinearFeedbackConstPtr>
    (opname, m_taskScoopLinearClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<TaskScoopLinearFeedbackConstPtr> (opname),
     default_action_done_cb<TaskScoopLinearResultConstPtr> (opname));
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

void OwlatInterface::panTiltCallback
(const owl_msgs::PanTiltPosition::ConstPtr& msg)
{
  m_pan_radians = msg->value[0];
  m_tilt_radians = msg->value[1];
  publish("PanRadians", m_pan_radians);
  publish("PanDegrees", m_pan_radians * R2D);
  publish("TiltRadians", m_tilt_radians);
  publish("TiltDegrees", m_tilt_radians * R2D);
}

Value OwlatInterface::getArmJointAngles() const
{
  return(Value(m_arm_joint_angles));
}

Value OwlatInterface::getArmJointAccelerations() const
{
  return(Value(m_arm_joint_accelerations));
}

Value OwlatInterface::getArmJointTorques() const
{
  return(Value(m_arm_joint_torques));
}

Value OwlatInterface::getArmJointVelocities() const
{
  return(Value(m_arm_joint_velocities));
}

Value OwlatInterface::getArmFTTorque() const
{
  return(Value(m_arm_ft_torque));
}

Value OwlatInterface::getArmFTForce() const
{
  return(Value(m_arm_ft_force));
}

Value OwlatInterface::getArmPose() const
{
  return(Value(m_arm_pose));
}

Value OwlatInterface::getArmTool() const
{
  return(Value(m_arm_tool));
}

Value OwlatInterface::getPanRadians() const
{
  return m_pan_radians;
}

Value OwlatInterface::getPanDegrees() const
{
  return m_pan_radians * R2D;
}

Value OwlatInterface::getTiltRadians() const
{
  return m_tilt_radians;
}

Value OwlatInterface::getTiltDegrees() const
{
  return m_tilt_radians * R2D;
}

Value OwlatInterface::getJointTelemetry (int joint, TelemetryType type) const
{
  if (joint >= 0 && joint < OwlatJoints) {
    switch (type) {
      case TelemetryType::Position: return m_arm_joint_angles[joint];
      case TelemetryType::Velocity: return m_arm_joint_velocities[joint];
      case TelemetryType::Effort: return m_arm_joint_torques[joint];
      case TelemetryType::Acceleration: return m_arm_joint_accelerations[joint];
    default:
      ROS_ERROR ("getJointTelemetry: unsupported telemetry type.");
    }
  }
  else {
    ROS_ERROR ("getJointTelemetry: invalid joint index %d", joint);
  }
  return 0;
}
