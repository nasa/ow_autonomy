#include "plexil_planner.h"
#include <ow_plexil/PlannerAction.h>

PlexilPlanner::PlexilPlanner() :
  m_plannerActionServer(m_genericNodeHandle, "selected_plans",boost::bind(&PlexilPlanner::executeCallback, this, _1), false)
{
    ROS_INFO("SETUP COMPLETE 1");
    m_plannerActionServer.start();
    ROS_INFO("SETUP COMPLETE");
}

void PlexilPlanner::executeCallback(const ow_plexil::PlannerGoalConstPtr &goal)
{
  ROS_INFO("MADE IT");
  ROS_INFO("%s", goal->command.c_str());
  m_feedback.progress = "GOING";
  m_plannerActionServer.publishFeedback(m_feedback);
  m_result.result = true;
  m_plannerActionServer.setSucceeded(m_result);
}
