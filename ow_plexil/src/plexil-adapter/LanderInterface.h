// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
// Exploration Research and Simulation can be found in README.md in
// the root directory of this repository.

#ifndef LanderInterface_H
#define LanderInterface_H

// Interface shared by OceanWATERS and JPL's OWLAT landers.

// ow_plexil
#include <Value.hh>
#include "PlexilInterface.h"
#include "joint_support.h"

// owl_msgs
#include <owl_msgs/ArmFindSurfaceAction.h>
#include <owl_msgs/PanTiltPosition.h>
#include <owl_msgs/ArmUnstowAction.h>
#include <owl_msgs/ArmStowAction.h>
#include <owl_msgs/ArmStopAction.h>
#include <owl_msgs/ArmMoveJointAction.h>
#include <owl_msgs/ArmMoveCartesianAction.h>
#include <owl_msgs/ArmMoveCartesianGuardedAction.h>

// ROS
#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Quaternion.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>

// C++
#include <string>
#include <memory>


using ArmUnstowActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmUnstowAction>;
using ArmStowActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmStowAction>;
using ArmStopActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmStopAction>;
using ArmTareFTSensorActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmTareFTSensorAction>;
using ArmSetToolActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmSetToolAction>;
using ArmMoveJointActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmMoveJointAction>;
using ArmMoveCartesianActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmMoveCartesianAction>;
using ArmMoveCartesianGuardedActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmMoveCartesianGuardedAction>;
using ArmFindSurfaceActionClient =
  actionlib::SimpleActionClient<owl_msgs::ArmFindSurfaceAction>;


class LanderInterface : public PlexilInterface
{
 public:
  static LanderInterface* instance();
  LanderInterface() = default;
  virtual ~LanderInterface() = 0 = default;
  LanderInterface (const LanderInterface&) = delete;
  LanderInterface& operator= (const LanderInterface&) = delete;

  void initialize();

  // Lander interface
  void armStow (int id);
  void armUnstow (int id);
  void armStop (int id);
  void armMoveJoint (bool relative, int joint, double angle, int id);
  void armMoveCartesian (int frame, bool relative,
                         const std::vector<double>& position,
                         const std::vector<double>& orientation,
                         int id);
  void armMoveCartesianGuarded (int frame, bool relative,
                                const std::vector<double>& position,
                                const std::vector<double>& orientation,
                                double force_threshold,
                                double torque_threshold,
                                int id);

 protected:
  template <class Client, class Goal, class ResultPtr, class FeedbackPtr>
  void nullaryAction (int id, const std::string& name,
                      std::unique_ptr<Client>& ac)
  {
    Goal goal;
    std::string opname = name;

    runAction<Client, Goal, ResultPtr, FeedbackPtr>
      (opname, ac, goal, id,
       default_action_active_cb (opname),
       default_action_feedback_cb<FeedbackPtr> (opname),
       default_action_done_cb<ResultPtr> (opname));
  }
  template<typename T>
  void connectActionServer (std::unique_ptr<actionlib::SimpleActionClient<T> >& c,
			    const std::string& name,
			    const std::string& topic = "")
  {
    if (! c->waitForServer(ros::Duration(ACTION_SERVER_TIMEOUT_SECS))) {
      ROS_ERROR ("%s action server did not connect!", name.c_str());
    }
    else if (topic != "") addSubscriber (topic, name);
  }

  void addSubscriber (const std::string& topic, const std::string& operation);

 private:
  // Actions
  void armFindSurfaceAction (int frame, bool relative,
                             const geometry_msgs::Point& pos,
                             const geometry_msgs::Vector3& normal,
                             double distance, double overdrive,
                             double force_threshold, double torque_threshold,
                             int id);
  void armMoveCartesianAction (int frame,
                               bool relative,
                               const geometry_msgs::Pose& pose,
                               int id);
  void armMoveCartesianGuardedAction (int frame,
                                      bool relative,
                                      const geometry_msgs::Pose&,
                                      double force_threshold,
                                      double torque_threshold,
                                      int id);
  void armMoveJointAction (bool relative, int joint, double angle, int id);

  // Action Clients
  std::unique_ptr<ArmStowActionClient> m_armStowClient;
  std::unique_ptr<ArmUnstowActionClient> m_armUnstowClient;
  std::unique_ptr<ArmStopActionClient> m_armStopClient;
  std::unique_ptr<ArmFindSurfaceActionClient> m_armFindSurfaceClient;
  std::unique_ptr<ArmMoveCartesianActionClient> m_armMoveCartesianClient;
  std::unique_ptr<ArmMoveCartesianGuardedActionClient> m_armMoveCartesianGuardedClient;
  std::unique_ptr<ArmMoveJointActionClient> m_armMoveJointClient;
};

#endif
