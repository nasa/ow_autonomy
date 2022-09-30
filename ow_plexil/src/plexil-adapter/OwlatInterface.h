// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
// Exploration Research and Simulation can be found in README.md in
// the root directory of this repository.

#ifndef OwlatInterface_H
#define OwlatInterface_H

// Interface to JPL's OWLAT simulator.

// C++
#include <memory>
#include <ros/ros.h>

// ow_plexil
#include <Value.hh>
#include "PlexilInterface.h"

// OWLAT Sim (installation required)
#include <owlat_sim_msgs/ARM_UNSTOWAction.h>
#include <owlat_sim_msgs/ARM_STOWAction.h>
#include <owlat_sim_msgs/ARM_MOVE_CARTESIANAction.h>
#include <owlat_sim_msgs/ARM_MOVE_CARTESIAN_GUARDEDAction.h>
#include <owlat_sim_msgs/ARM_MOVE_JOINTAction.h>
#include <owlat_sim_msgs/ARM_MOVE_JOINTSAction.h>
#include <owlat_sim_msgs/ARM_MOVE_JOINTS_GUARDEDAction.h>
#include <owlat_sim_msgs/ARM_PLACE_TOOLAction.h>
#include <owlat_sim_msgs/ARM_SET_TOOLAction.h>
#include <owlat_sim_msgs/ARM_STOPAction.h>
#include <owlat_sim_msgs/ARM_TARE_FSAction.h>
#include <owlat_sim_msgs/TASK_PSPAction.h>
#include <owlat_sim_msgs/TASK_SCOOPAction.h>
#include <owlat_sim_msgs/ARM_JOINT_ANGLES.h>
#include <owlat_sim_msgs/ARM_JOINT_ACCELERATIONS.h>
#include <owlat_sim_msgs/ARM_JOINT_TORQUES.h>
#include <owlat_sim_msgs/ARM_JOINT_VELOCITIES.h>
#include <owlat_sim_msgs/ARM_FT_TORQUE.h>
#include <owlat_sim_msgs/ARM_FT_FORCE.h>
#include <owlat_sim_msgs/ARM_POSE.h>
#include <owlat_sim_msgs/ARM_TOOL.h>

using OwlatUnstowActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_UNSTOWAction>;
using OwlatStowActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_STOWAction>;
using OwlatArmMoveCartesianActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_MOVE_CARTESIANAction>;
using OwlatArmMoveCartesianGuardedActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_MOVE_CARTESIAN_GUARDEDAction>;
using OwlatArmMoveJointActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_MOVE_JOINTAction>;
using OwlatArmMoveJointsActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_MOVE_JOINTSAction>;
using OwlatArmMoveJointsGuardedActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_MOVE_JOINTS_GUARDEDAction>;
using OwlatArmPlaceToolActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_PLACE_TOOLAction>;
using OwlatArmSetToolActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_SET_TOOLAction>;
using OwlatArmStopActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_STOPAction>;
using OwlatArmTareFSActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_TARE_FSAction>;
using OwlatTaskPSPActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::TASK_PSPAction>;
using OwlatTaskScoopActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::TASK_SCOOPAction>;

class OwlatInterface : public PlexilInterface
{
 public:
  static OwlatInterface* instance();
  OwlatInterface() = default;
  ~OwlatInterface() = default;
  OwlatInterface (const OwlatInterface&) = delete;
  OwlatInterface& operator= (const OwlatInterface&) = delete;

  void initialize();

  // Lander interface
  void owlatUnstow (int id);
  void owlatStow (int id);
  void owlatArmMoveCartesian (int frame, bool relative, 
                              const std::vector<double>& position, 
                              const std::vector<double>& orientation, 
                              int id);
  void owlatArmMoveCartesianGuarded (int frame, bool relative, 
                                     const std::vector<double>& position, 
                                     const std::vector<double>& orientation,
                                     bool retracting,
                                     double force_threshold,
                                     double torque_threshold,int id);
  void owlatArmMoveJoint (bool relative, int joint, double angle, 
                          int id);
  void owlatArmMoveJoints (bool relative,
                           const std::vector<double>& angles,
                           int id); 
  void owlatArmMoveJointsGuarded (bool relative,
                                  const std::vector<double>& angles, 
                                  bool retracting, double force_threshold,
                                  double torque_threshold, int id);
  void owlatArmPlaceTool (int frame,
                          bool relative,
                          const std::vector<double>& position, 
                          const std::vector<double>& normal,
                          double distance, double overdrive,
                          bool retracting, double force_threshold,
                          double torque_threshold, int id);
  void owlatArmSetTool (int tool, int id);
  void owlatArmStop (int id);
  void owlatArmTareFS (int id);
  void owlatTaskPSP (int frame, bool relative, const std::vector<double>& point, 
                     const std::vector<double>& normal, double max_depth,
                     double max_force, int id); 
  void owlatTaskScoop (int frame, bool relative, const std::vector<double>& point, 
                       const std::vector<double>& normal, int id); 

  // Lookups
  PLEXIL::Value getArmJointAngles();
  PLEXIL::Value getArmJointAccelerations();
  PLEXIL::Value getArmJointTorques();
  PLEXIL::Value getArmJointVelocities();
  PLEXIL::Value getArmFTTorque();
  PLEXIL::Value getArmFTForce();
  PLEXIL::Value getArmPose();
  PLEXIL::Value getArmTool();
  PLEXIL::Value getPSPStopReason();

 private:

  // Actions
  void owlatUnstowAction (int id);
  void owlatStowAction (int id);
  void owlatArmMoveCartesianAction (int frame, bool relative, 
                                    const std::vector<double>& position, 
                                    const std::vector<double>& orientation, 
                                    int id);
  void owlatArmMoveCartesianGuardedAction (int frame, bool relative, 
                                           const std::vector<double>& position, 
                                           const std::vector<double>& orientation,
                                           bool retracting, 
                                           double force_threshold, 
                                           double torque_threshold,int id);
  void owlatArmMoveJointAction (bool relative, int joint,
                                double angle, int id); 
  void owlatArmMoveJointsAction (bool relative, const std::vector<double>& angles,
                                 int id);
  void owlatArmMoveJointsGuardedAction (bool relative,
                                        const std::vector<double>& angles, 
                                        bool retracting, double force_threshold,
                                        double torque_threshold, int id);
  void owlatArmPlaceToolAction (int frame, bool relative, 
                                const std::vector<double>& position,
                                const std::vector<double>& normal,
                                double distance, double overdrive,
                                bool retracting, double force_threshold,
                                double torque_threshold, int id);
  void owlatArmSetToolAction (int tool, int id);
  void owlatArmStopAction (int id);
  void owlatArmTareFSAction (int id);
  void owlatTaskPSPAction (int frame, bool relative,
                           const std::vector<double>& point, 
                           const std::vector<double>& normal, double max_depth,
                           double max_force, int id);
  void owlatTaskScoopAction (int frame, bool relative,
                             const std::vector<double>& point, 
                             const std::vector<double>& normal, int id); 

  // Node handle
  std::unique_ptr<ros::NodeHandle> m_genericNodeHandle;

  // Subscribers
  std::unique_ptr<ros::Subscriber> m_armJointAnglesSubscriber;
  std::unique_ptr<ros::Subscriber> m_armJointAccelerationsSubscriber;
  std::unique_ptr<ros::Subscriber> m_armJointTorquesSubscriber;
  std::unique_ptr<ros::Subscriber> m_armJointVelocitiesSubscriber;
  std::unique_ptr<ros::Subscriber> m_armFTTorqueSubscriber;
  std::unique_ptr<ros::Subscriber> m_armFTForceSubscriber;
  std::unique_ptr<ros::Subscriber> m_armPoseSubscriber;
  std::unique_ptr<ros::Subscriber> m_armToolSubscriber;

  // Callbacks
  void armJointAnglesCallback
  (const owlat_sim_msgs::ARM_JOINT_ANGLES::ConstPtr& msg);
  void armJointAccelerationsCallback
  (const owlat_sim_msgs::ARM_JOINT_ACCELERATIONS::ConstPtr&);
  void armJointTorquesCallback
  (const owlat_sim_msgs::ARM_JOINT_TORQUES::ConstPtr& msg);
  void armJointVelocitiesCallback
  (const owlat_sim_msgs::ARM_JOINT_VELOCITIES::ConstPtr& msg);
  void armFTTorqueCallback(const owlat_sim_msgs::ARM_FT_TORQUE::ConstPtr& msg);
  void armFTForceCallback(const owlat_sim_msgs::ARM_FT_FORCE::ConstPtr& msg);
  void armPoseCallback(const owlat_sim_msgs::ARM_POSE::ConstPtr& msg);
  void armToolCallback(const owlat_sim_msgs::ARM_TOOL::ConstPtr& msg);

  // Action Clients
  std::unique_ptr<OwlatUnstowActionClient> m_owlatUnstowClient;
  std::unique_ptr<OwlatStowActionClient> m_owlatStowClient;
  std::unique_ptr<OwlatArmMoveCartesianActionClient> m_owlatArmMoveCartesianClient;
  std::unique_ptr<OwlatArmMoveCartesianGuardedActionClient> m_owlatArmMoveCartesianGuardedClient;
  std::unique_ptr<OwlatArmMoveJointActionClient> m_owlatArmMoveJointClient;
  std::unique_ptr<OwlatArmMoveJointsActionClient> m_owlatArmMoveJointsClient;
  std::unique_ptr<OwlatArmMoveJointsGuardedActionClient> m_owlatArmMoveJointsGuardedClient;
  std::unique_ptr<OwlatArmPlaceToolActionClient> m_owlatArmPlaceToolClient;
  std::unique_ptr<OwlatArmSetToolActionClient> m_owlatArmSetToolClient;
  std::unique_ptr<OwlatArmStopActionClient> m_owlatArmStopClient;
  std::unique_ptr<OwlatArmTareFSActionClient> m_owlatArmTareFSClient;
  std::unique_ptr<OwlatTaskPSPActionClient> m_owlatTaskPSPClient;
  std::unique_ptr<OwlatTaskScoopActionClient> m_owlatTaskScoopClient;

  // Member variables
  std::vector<double> m_arm_joint_angles;
  std::vector<double> m_arm_joint_accelerations;
  std::vector<double> m_arm_joint_torques;
  std::vector<double> m_arm_joint_velocities;
  std::vector<double> m_arm_ft_torque;
  std::vector<double> m_arm_ft_force;
  std::vector<double> m_arm_pose;
  double m_arm_tool;
};

#endif
