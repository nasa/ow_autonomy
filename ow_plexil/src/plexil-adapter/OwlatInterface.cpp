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

const string Name_ArmFindSurface       = "ArmFindSurface";
const string Name_ArmMoveJoints        = "ArmMoveJoints";
const string Name_ArmMoveJointsGuarded = "ArmMoveJointsGuarded";
const string Name_Tare                 = "ArmTareFTSensor";
const string Name_ArmSetTool           = "ArmSetTool";
const string Name_TaskPSP              = "TaskPSP";
const string Name_Discard              = "TaskDiscardSample";
const string Name_ShearBevameter       = "TaskShearBevameter";
const string Name_Penetrometer         = "TaskPenetrometer";
const string Name_ScoopCircular        = "TaskScoopCircular";
const string Name_ScoopLinear          = "TaskScoopLinear";

const size_t OwlatJoints = 6;

static vector<string> LanderOpNames = {
  Name_ArmFindSurface,
  Name_Discard,
  Name_ArmMoveJoints,
  Name_ArmMoveJointsGuarded,
  Name_ArmSetTool,
  Name_Tare,
  Name_TaskPSP,
  Name_ShearBevameter,
  Name_Penetrometer,
  Name_ScoopCircular,
  Name_ScoopLinear
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
    m_discardSampleClient =
      make_unique<TaskDiscardSampleActionClient>(Name_Discard, true);
    m_armMoveJointsClient =
      make_unique<ArmMoveJointsActionClient>(Name_ArmMoveJoints, true);
    m_armMoveJointsGuardedClient =
      make_unique<ArmMoveJointsGuardedActionClient> (Name_ArmMoveJointsGuarded,
                                                     true);
    m_armSetToolClient =
      make_unique<ArmSetToolActionClient>(Name_ArmSetTool, true);
    m_armTareFTSensorClient =
      make_unique<ArmTareFTSensorActionClient>(Name_Tare, true);
    m_taskPSPClient =
      make_unique<TaskPSPActionClient>(Name_TaskPSP, true);
    m_taskShearBevameterClient =
      make_unique<TaskShearBevameterActionClient>(Name_ShearBevameter, true);
    m_taskPenetrometerClient =
      make_unique<TaskPenetrometerActionClient>(Name_Penetrometer, true);
    m_taskScoopCircularClient =
      make_unique<TaskScoopCircularActionClient>(Name_ScoopCircular, true);
    m_taskScoopLinearClient =
      make_unique<TaskScoopLinearActionClient>(Name_ScoopLinear, true);

    // Connect to action servers
    connectActionServer (m_armFindSurfaceClient, Name_ArmFindSurface,
                         "/ArmFindSurface/status");
    connectActionServer (m_discardSampleClient, Name_Discard,
                         "/TaskDiscardSample/status");
    connectActionServer (m_armMoveJointsClient, Name_ArmMoveJoints,
                         "/TaskArmMoveJoints/status");
    connectActionServer (m_armMoveJointsGuardedClient, Name_ArmMoveJointsGuarded,
                         "/TaskArmMoveJointsGuarded/status");
    connectActionServer (m_armSetToolClient, Name_ArmSetTool,
                         "/ArmSetTool/status");
    connectActionServer (m_armTareFTSensorClient, Name_Tare,
                         "/ArmTareFTSensor/status");
    connectActionServer (m_taskShearBevameterClient, Name_ShearBevameter,
                         "/TaskShearBevameter/status");
    connectActionServer (m_taskPenetrometerClient, Name_Penetrometer,
                         "/TaskPenetrometer/status");
    connectActionServer (m_taskScoopLinearClient, Name_ScoopLinear,
                         "/TaskScoopLinear/status");
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
                                           double force_threshold,
					   double torque_threshold,
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
    (Name_Discard, m_discardSampleClient, goal, id,
     default_action_active_cb (Name_Discard),
     default_action_feedback_cb<TaskDiscardSampleFeedbackConstPtr> (Name_Discard),
     default_action_done_cb<TaskDiscardSampleResultConstPtr> (Name_Discard));
}


void OwlatInterface::armMoveJoints (bool relative,
                                    const vector<double>& angles,
                                    int id)
{
  if (! markOperationRunning (Name_ArmMoveJoints, id)) return;
  thread action_thread (&OwlatInterface::armMoveJointsAction,
                        this, relative, angles, id);
  action_thread.detach();
}

void OwlatInterface::armMoveJointsAction (bool relative,
                                          const vector<double>& angles,
                                          int id)
{
  ArmMoveJointsGoal goal;
  goal.relative = relative;
  for(int i = 0; i < goal.angles.size(); i++){
    goal.angles[i] = angles[i];
  }
  string opname = Name_ArmMoveJoints;

  runAction<actionlib::SimpleActionClient<ArmMoveJointsAction>,
            ArmMoveJointsGoal,
            ArmMoveJointsResultConstPtr,
            ArmMoveJointsFeedbackConstPtr>
    (opname, m_armMoveJointsClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ArmMoveJointsFeedbackConstPtr> (opname),
     default_action_done_cb<ArmMoveJointsResultConstPtr> (opname));
}

void OwlatInterface::armMoveJointsGuarded (bool relative,
                                           const vector<double>& angles,
                                           double force_threshold,
                                           double torque_threshold,
                                           int id)
{
  if (! markOperationRunning (Name_ArmMoveJointsGuarded, id)) return;
  thread action_thread (&OwlatInterface::armMoveJointsGuardedAction,
                        this, relative, angles, force_threshold,
                        torque_threshold, id);
  action_thread.detach();
}

void OwlatInterface::armMoveJointsGuardedAction (bool relative,
                                                 const vector<double>& angles,
                                                 double force_threshold,
                                                 double torque_threshold,
                                                 int id)
{

  ArmMoveJointsGuardedGoal goal;
  goal.relative = relative;
  for(int i = 0; i < goal.angles.size(); i++){
    goal.angles[i] = angles[i];
  }
  goal.force_threshold = force_threshold;
  goal.torque_threshold = torque_threshold;
  string opname = Name_ArmMoveJointsGuarded;

  runAction<actionlib::SimpleActionClient<ArmMoveJointsGuardedAction>,
            ArmMoveJointsGuardedGoal,
            ArmMoveJointsGuardedResultConstPtr,
            ArmMoveJointsGuardedFeedbackConstPtr>
    (opname, m_armMoveJointsGuardedClient, goal, id,
     default_action_active_cb (opname),
     default_action_feedback_cb<ArmMoveJointsGuardedFeedbackConstPtr> (opname),
     default_action_done_cb<ArmMoveJointsGuardedResultConstPtr> (opname));
}

void OwlatInterface::armSetTool (int tool, int id)
{
  if (! markOperationRunning (Name_ArmSetTool, id)) return;
  thread action_thread (&OwlatInterface::armSetToolAction, this, tool, id);
  action_thread.detach();
}

void OwlatInterface::armSetToolAction (int tool, int id)
{
  ArmSetToolGoal goal;
  goal.tool = tool;
  string opname = Name_ArmSetTool;

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
  thread action_thread (&OwlatInterface::runNullaryAction<
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
  if (! markOperationRunning (Name_ShearBevameter, id)) return;
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
  if (! markOperationRunning (Name_Penetrometer, id)) return;
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
  for(int i = 0; i < goal.point.size(); i++) {
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
  string opname = Name_ShearBevameter;

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
  string opname = Name_Penetrometer;

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
  if (! markOperationRunning (Name_ScoopCircular, id)) return;
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
  string opname = Name_ScoopCircular;

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
  if (! markOperationRunning (Name_ScoopLinear, id)) return;
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
  string opname = Name_ScoopLinear;

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
