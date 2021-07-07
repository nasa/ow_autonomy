#include "plexil_planner.h"
#include <string>
#include <ow_plexil/PlannerAction.h>
#include <actionlib/server/simple_action_server.h>
#include "OwExecutive.h"
#include "OwInterface.h"

PlexilPlanner::PlexilPlanner() :
  m_plannerActionServer(m_genericNodeHandle,"selected_plans",boost::bind(&PlexilPlanner::executeCallback, this, _1), false)
{

  m_executive = std::make_unique<OwExecutive>();
  if (! m_executive->instance()->initialize()) {
    ROS_ERROR("Could not initialize OW executive, shutting down.");
  }
  m_interface = std::make_unique<OwInterface>();
  m_interface->instance()->initialize();

  // wait for the first proper clock message before running the plan
  ros::Rate warmup_rate(0.1);
  ros::Time begin = ros::Time::now();
  while (ros::Time::now() - begin == ros::Duration(0.0))
  {
    ros::spinOnce();
    warmup_rate.sleep();
  }

  firstPlan = true;
  
  m_plannerActionServer.start();
  ROS_INFO("Planning Action Server Started");
}

void PlexilPlanner::executeCallback(const ow_plexil::PlannerGoalConstPtr &goal)
{
  ROS_INFO("%s", goal->command.c_str());

  if(goal->command.compare("ADD") == 0){
    planArray.insert(planArray.end(), goal->plans.begin(), goal->plans.end());
  }
  else if(goal->command.compare("RESET") == 0){
    planArray.clear();
    if(m_executive->instance()->reset() == true){
      ROS_INFO ("Plan list reset.");
    }
    else{
      m_result.result = false;
      m_plannerActionServer.setAborted(m_result);
    }
  }
  else{
    ROS_ERROR("Command %s not recognized", goal->command.c_str());
  }

  ros::Rate rate(10); // 1 Hz seems appropriate, for now.
  ros::Rate rate2(1); // 1 Hz seems appropriate, for now.
  int length = planArray.size();
  for(int i = 0; i < length; i++){
    while(!m_executive->instance()->getPlanState() && firstPlan == false){
          ros::spinOnce();
          if(m_plannerActionServer.isPreemptRequested() || !ros::ok()){
            m_plannerActionServer.setPreempted();
            return;
          }
          rate2.sleep();
        }

    if(m_executive->instance()->runPlan(planArray[0].c_str())){
        if(firstPlan == true){
          firstPlan = false; 
        }
        ROS_INFO ("Running plan %s", planArray[0].c_str());
        int timeout = 0;
        // Times out after 5 seconds or the plan is registered as running.
        while(m_executive->instance()->getPlanState() && timeout < 50){
          ros::spinOnce();
          rate.sleep();
          timeout+=1;
          if(timeout % 10 == 0){
            ROS_ERROR("Plan not responding, timing out in %i seconds", (5 - timeout/10));
          }
        }
        if(timeout == 50){
          ROS_INFO ("Plan timed out, try again.");
          m_feedback.progress = "FAILED:" + planArray[0];
          m_plannerActionServer.publishFeedback(m_feedback);
        }
        else{
          m_feedback.progress = "SUCCESS:" + planArray[0];
          m_plannerActionServer.publishFeedback(m_feedback);
        }
    }
    else{
      m_feedback.progress = "FAILED:" + planArray[0];
      m_plannerActionServer.publishFeedback(m_feedback);
    }
    planArray.erase(planArray.begin());
  }
  m_result.result = true;
  m_plannerActionServer.setSucceeded(m_result);
}

