// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Plexil_Planner_H
#define Plexil_Planner_H

#include <memory>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ow_plexil/PlannerAction.h>
#include "OwExecutive.h"
#include "OwInterface.h"

class PlexilPlanner{
  public:
    PlexilPlanner();
    void executeCallback(const ow_plexil::PlannerGoalConstPtr&);
    void start();

  
  private:
    ros::NodeHandle m_genericNodeHandle;
    std::unique_ptr<OwExecutive> m_executive;
    actionlib::SimpleActionServer<ow_plexil::PlannerAction> m_plannerActionServer;
    ow_plexil::PlannerFeedback m_feedback;
    ow_plexil::PlannerResult m_result;
    std::vector<std::string> plan_array;
    std::vector<std::string> callback_array;
    bool firstPlan;
 
};

#endif

