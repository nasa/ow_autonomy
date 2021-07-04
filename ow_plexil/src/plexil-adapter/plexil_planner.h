// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Plexil_Planner_H
#define Plexil_Planner_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ow_plexil/PlannerAction.h>

class PlexilPlanner{
  public:
    PlexilPlanner();
    void executeCallback(const ow_plexil::PlannerGoalConstPtr&);
  
  private:
    ros::NodeHandle m_genericNodeHandle;
    actionlib::SimpleActionServer<ow_plexil::PlannerAction> m_plannerActionServer;
    ow_plexil::PlannerFeedback m_feedback;
    ow_plexil::PlannerResult m_result;
 
};

#endif

