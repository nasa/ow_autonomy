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

using std::string;
using std::vector;

using OwlatUnstowActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_UNSTOWAction>;

using OwlatStowActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_STOWAction>;

using OwlatArmMoveCartesianActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_MOVE_CARTESIANAction>;

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
 private:
  void owlatUnstowAction (int id);
  void owlatStowAction (int id);
  void owlatArmMoveCartesianAction (int frame, bool relative, 
                                    vector<double> position, 
                                    vector<double> orientation, int id);

  static std::shared_ptr<OwlatInterface> m_instance;
  std::unique_ptr<OwlatUnstowActionClient> m_owlatUnstowClient;
  std::unique_ptr<OwlatStowActionClient> m_owlatStowClient;
  std::unique_ptr<OwlatArmMoveCartesianActionClient> m_owlatArmMoveCartesianClient;
};

#endif
