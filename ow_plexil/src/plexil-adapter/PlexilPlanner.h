// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Plexil_Planner_H
#define Plexil_Planner_H

#include <ros/ros.h>
#include <ow_plexil/PlannerCommand.h>
#include "OwExecutive.h"
#include "OwInterface.h"

class PlexilPlanner{
  public:
    PlexilPlanner();
    void plannerCommandsCallback(const ow_plexil::PlannerCommand::ConstPtr&);
    void start();
    void initialize();

  
  private:
    std::unique_ptr<ros::NodeHandle> m_genericNodeHandle;
    std::unique_ptr<OwExecutive> m_executive;
    std::unique_ptr<ros::Subscriber> m_plannerCommandSubscriber;
    std::unique_ptr<ros::Publisher> m_planStatusPublisher;
    std::vector<std::string> plan_array;
    bool first_plan;
 
};

#endif

