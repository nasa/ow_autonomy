// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef OwlatInterface_H
#define OwlatInterface_H

// Interface to JPL's OWLAT simulator.

// C++
#include <memory>
#include <ros/ros.h>

// ow_plexil
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
#include <owlat_sim_msgs/TASK_DROPOFFAction.h>
#include <owlat_sim_msgs/TASK_PSPAction.h>
#include <owlat_sim_msgs/TASK_SCOOPAction.h>
#include <owlat_sim_msgs/TASK_SHEAR_BEVAMETERAction.h>

using std::string;
using std::vector;

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
using OwlatTaskDropoffActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::TASK_DROPOFFAction>;
using OwlatTaskPSPActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::TASK_PSPAction>;
using OwlatTaskScoopActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::TASK_SCOOPAction>;
using OwlatTaskShearBevameterActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::TASK_SHEAR_BEVAMETERAction>;

class OwlatInterface : public PlexilInterface
{
 public:
  static std::shared_ptr<OwlatInterface> instance();
  OwlatInterface() = default;
  ~OwlatInterface() = default;
  OwlatInterface (const OwlatInterface&) = delete;
	OwlatInterface& operator= (const OwlatInterface&) = delete;

  void initialize();

  // Lander interface
  void owlatUnstow (int id);
  void owlatStow (int id);
  void owlatArmMoveCartesian (int frame, bool relative, 
                              vector<double> position, 
                              vector<double> orientation, int id);
  void owlatArmMoveCartesianGuarded (int frame, bool relative, 
                                     vector<double> position, 
                                     vector<double> orientation,
                                     bool retracting,
                                     double force_threshold,
                                     double torque_threshold,int id);
  void owlatArmMoveJoint (bool relative, int joint, double angle, 
                          int id);
  void owlatArmMoveJoints (bool relative, vector<double> joints, int id); 
  void owlatArmMoveJointsGuarded (bool relative, vector<double> joints, 
                                  bool retracting, double force_threshold,
                                  double torque_threshold, int id);
  void owlatArmPlaceTool (int frame, bool relative, vector<double> position, 
                          vector<double> normal,double distance, double overdrive,
                          bool retracting, double force_threshold,
                          double torque_threshold, int id);
  void owlatArmSetTool (int tool, int id);
  void owlatArmStop (int id);
  void owlatArmTareFS (int id);
  void owlatTaskDropoff (int frame, bool relative,vector<double> point, int id);
  void owlatTaskPSP (int frame, bool relative, vector<double> point, 
                     vector<double> normal, double max_depth, float max_force,
                     int id); 
  void owlatTaskScoop (int frame, bool relative, vector<double> point, 
                       vector<double> normal, int id); 
  void owlatTaskShearBevameter (int frame, bool relative, vector<double> point, 
                                vector<double> normal, double preload,
                                double max_torque, int id); 



 private:
  void owlatUnstowAction (int id);
  void owlatStowAction (int id);
  void owlatArmMoveCartesianAction (int frame, bool relative, 
                                    vector<double> position, 
                                    vector<double> orientation, int id);
  void owlatArmMoveCartesianGuardedAction (int frame, bool relative, 
                                           vector<double> position, 
                                           vector<double> orientation,
                                           bool retracting, 
                                           double force_threshold, 
                                           double torque_threshold,int id);
  void owlatArmMoveJointAction (bool relative, int joint,
                                double angle, int id); 
  void owlatArmMoveJointsAction (bool relative, vector<double> joints, int id);
  void owlatArmMoveJointsGuardedAction (bool relative, vector<double> joints, 
                                        bool retracting, double force_threshold,
                                        double torque_threshold, int id);
  void owlatArmPlaceToolAction (int frame, bool relative, 
                                vector<double> position, vector<double> normal,
                                double distance, double overdrive,
                                bool retracting, double force_threshold,
                                double torque_threshold, int id);
  void owlatArmSetToolAction (int tool, int id);
  void owlatArmStopAction (int id);
  void owlatArmTareFSAction (int id);
  void owlatTaskDropoffAction (int frame, bool relative,
                               vector<double> point, int id);
  void owlatTaskPSPAction (int frame, bool relative, vector<double> point, 
                           vector<double> normal, double max_depth,
                           double max_force, int id);
  void owlatTaskScoopAction (int frame, bool relative, vector<double> point, 
                             vector<double> normal, int id); 
  void owlatTaskShearBevameterAction (int frame, bool relative,
                                      vector<double> point, vector<double> normal,
                                      double preload, double max_torque, int id); 



  static std::shared_ptr<OwlatInterface> m_instance;
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
  std::unique_ptr<OwlatTaskDropoffActionClient> m_owlatTaskDropoffClient;
  std::unique_ptr<OwlatTaskPSPActionClient> m_owlatTaskPSPClient;
  std::unique_ptr<OwlatTaskScoopActionClient> m_owlatTaskScoopClient;
  std::unique_ptr<OwlatTaskShearBevameterActionClient> m_owlatTaskShearBevameterClient;

};

#endif
