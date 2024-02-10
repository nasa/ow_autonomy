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

OwlatInterface::OwlatInterface ()
  : m_arm_tool (0)
{
    m_end_effector_ft.resize(6);
}

void OwlatInterface::initialize()
{
  LanderInterface::initialize();

  for (auto name : LanderOpNames) {
    registerLanderOperation (name);
  }

  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle ->
      subscribe("/system_faults_status", QueueSize,
                &OwlatInterface::systemFaultMessageCallback, this)));

  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle ->
      subscribe("/owlat_sim/ARM_TOOL", QueueSize,
                &OwlatInterface::armToolCallback, this)));

  m_subscribers.push_back
    (make_unique<ros::Subscriber>
     (m_genericNodeHandle ->
      subscribe("/arm_end_effector_force_torque", QueueSize,
                &OwlatInterface::ftCallback, this)));

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
  connectActionServer (m_armFindSurfaceClient, Name_ArmFindSurface);
  connectActionServer (m_discardSampleClient, Name_Discard);
  connectActionServer (m_armMoveJointsClient, Name_ArmMoveJoints);
  connectActionServer (m_armMoveJointsGuardedClient, Name_ArmMoveJointsGuarded);
  connectActionServer (m_armSetToolClient, Name_ArmSetTool);
  connectActionServer (m_armTareFTSensorClient, Name_Tare);
  connectActionServer (m_taskShearBevameterClient, Name_ShearBevameter);
  connectActionServer (m_taskPenetrometerClient, Name_Penetrometer);
  connectActionServer (m_taskScoopLinearClient, Name_ScoopLinear);
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

void OwlatInterface::systemFaultMessageCallback
(const owl_msgs::SystemFaultsStatus::ConstPtr& msg)
{
  updateFaultStatus (msg->value, m_systemErrors, "SYSTEM", "SystemFault");
}

void OwlatInterface::ftCallback
(const owl_msgs::ArmEndEffectorForceTorque::ConstPtr& msg)
{
  copy(msg->value.begin(), msg->value.end(), m_end_effector_ft.begin());
  publish("ArmEndEffectorForceTorque", m_end_effector_ft);
}

void OwlatInterface::armToolCallback
// NOTE that this older message file is still used (see header).
(const owlat_sim_msgs::ARM_TOOL::ConstPtr& msg)
{
  m_arm_tool = msg->value.value;
  publish("ArmTool", m_arm_tool);
}

Value OwlatInterface::getArmTool() const
{
  return(Value(m_arm_tool));
}

bool OwlatInterface::systemFault () const
{
  return faultActive (m_systemErrors);
}

bool OwlatInterface::armGoalError () const
{
  return m_systemErrors.at("ArmGoalError").second;
}

bool OwlatInterface::cameraGoalError () const
{
  return m_systemErrors.at("CameraGoalError").second;
}

bool OwlatInterface::panTiltGoalError () const
{
  return m_systemErrors.at("PanTiltGoalError").second;
}

bool OwlatInterface::drillGoalError () const
{
  return m_systemErrors.at("DrillGoalError").second;
}

bool OwlatInterface::taskGoalError () const
{
  return m_systemErrors.at("TaskGoalError").second;
}

vector<double> OwlatInterface::getArmEndEffectorFT () const
{
  return m_end_effector_ft;
}
