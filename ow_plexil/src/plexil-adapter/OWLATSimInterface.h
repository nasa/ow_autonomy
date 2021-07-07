// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef OwlatSimInterface_H
#define OwlatSimInterface_H

// Interface to JPL's OWLAT simulator.

// C++
#include <memory>
#include <ros/ros.h>

// ow_plexil
#include "PlexilInterface.h"

// OWLAT Sim (installation required)
#include <owlat_sim_msgs/ARM_UNSTOWAction.h>

using OwlatUnstowActionClient =
  actionlib::SimpleActionClient<owlat_sim_msgs::ARM_UNSTOWAction>;

class OWLATSimInterface : public PlexilInterface
{
 public:
  OWLATSimInterface() = default;
  ~OWLATSimInterface() = default;
  OWLATSimInterface (const OWLATSimInterface&) = delete;
	OWLATSimInterface& operator= (const OWLATSimInterface&) = delete;

  void initialize();

  // Lander interface
  void owlatUnstow (int id);

 private:
  void owlatUnstowAction (int id);

  std::unique_ptr<OwlatUnstowActionClient> m_owlatUnstowClient;
};

#endif
