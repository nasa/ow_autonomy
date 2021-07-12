// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Plexil_Planner_H
#define Plexil_Planner_H

#include <ros/ros.h>
#include <ow_plexil/PlanSelectionCommand.h>
#include "OwExecutive.h"
#include "OwInterface.h"

class PlexilPlanSelection{
  public:
    PlexilPlanSelection() = default;
    ~PlexilPlanSelection() = default;
    PlexilPlanSelection(const PlexilPlanSelection&) = delete;
    PlexilPlanSelection& operator = (const PlexilPlanSelection&) = delete;
    void planSelectionCommandsCallback(const ow_plexil::PlanSelectionCommand::ConstPtr&);
    void initialize(std::string);
    void start();

  
  private:
    std::unique_ptr<ros::NodeHandle> m_genericNodeHandle;
    std::unique_ptr<OwExecutive> m_executive;
    std::unique_ptr<ros::Subscriber> m_planSelectionCommandSubscriber;
    std::unique_ptr<ros::Publisher> m_planSelectionStatusPublisher;
    std::vector<std::string> plan_array;
    bool m_first_plan;
 
};

#endif

