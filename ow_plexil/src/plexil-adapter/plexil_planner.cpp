#include "plexil_planner.h"
#include <string>
#include <ow_plexil/PlannerAction.h>
#include <actionlib/server/simple_action_server.h>
#include "OwExecutive.h"
#include "OwInterface.h"

PlexilPlanner::PlexilPlanner() :
  m_plannerActionServer(m_genericNodeHandle,"selected_plans",boost::bind(&PlexilPlanner::executeCallback, this, _1), false)
{
  ROS_INFO("Starting planning action server...");
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
  ROS_INFO("Planning action server started, ready for PLEXIL plans.");
}

void PlexilPlanner::executeCallback(const ow_plexil::PlannerGoalConstPtr &goal)
{

  if(goal->command.compare("ADD") == 0){
    planArray.insert(planArray.end(), goal->plans.begin(), goal->plans.end());
  }
  else if(goal->command.compare("RESET") == 0){
    planArray.clear();
    ROS_INFO ("Plan list reset, current plan will finish execution before stopping");
  }
  else{
    ROS_ERROR("Command %s not recognized", goal->command.c_str());
  }

  ros::Rate rate(10); // 1 Hz seems appropriate, for now.
  ros::Rate rate2(1); // 1 Hz seems appropriate, for now.
  int length = planArray.size();

  for(int i = 0; i < length; i++){
    while(!m_executive->instance()->getPlanState() && firstPlan == false){
          //ros::spinOnce();
          std::cout << "BEGIN ";
          std::cout << m_executive->instance()->getPlanState() << std::endl;
          if(m_plannerActionServer.isPreemptRequested() || !ros::ok()){
            m_plannerActionServer.setPreempted();
            return;
          }
          rate2.sleep();
        }
    m_feedback.progress = "COMPLETE";
    m_plannerActionServer.publishFeedback(m_feedback);

    if(m_executive->instance()->runPlan(planArray[0].c_str())){
        if(firstPlan == true){
          firstPlan = false; 
        }
        int timeout = 0;
        // Times out after 3 seconds or the plan is registered as running.
        while(m_executive->instance()->getPlanState() && timeout < 30){
          std::cout << "MIDDLE ";
          std::cout << m_executive->instance()->getPlanState() << std::endl;
          //ros::spinOnce();
          rate.sleep();
          timeout+=1;
          if(timeout % 10 == 0){
            ROS_ERROR("Plan not responding, timing out in %i seconds", (3 - timeout/10));
          }
        }
        if(timeout == 30){
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

  while(!m_executive->instance()->getPlanState() && firstPlan == false){
    //ros::spinOnce();
    std::cout << "END ";
    std::cout << m_executive->instance()->getPlanState() << std::endl;
    if(m_plannerActionServer.isPreemptRequested() || !ros::ok()){
      m_plannerActionServer.setPreempted();
      return;
    }
    rate2.sleep();
  }

  m_feedback.progress = "COMPLETE";
  m_plannerActionServer.publishFeedback(m_feedback);
  m_result.result = true;
  m_plannerActionServer.setSucceeded(m_result);
}

