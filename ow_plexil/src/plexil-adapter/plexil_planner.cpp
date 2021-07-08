#include "plexil_planner.h"
#include <string>
#include <ow_plexil/PlannerAction.h>
#include <actionlib/server/simple_action_server.h>
#include "OwExecutive.h"
#include "OwInterface.h"

PlexilPlanner::PlexilPlanner() :
  m_plannerActionServer(m_genericNodeHandle,"selected_plans",boost::bind(&PlexilPlanner::executeCallback, this, _1), false)
{
}

void PlexilPlanner::start()
{
  ROS_INFO("Starting planning action server...");
  m_executive.reset(OwExecutive::instance());
  if (! m_executive->initialize()) {
    ROS_ERROR("Could not initialize OW executive, shutting down.");
  }

  OwInterface::instance()->initialize();
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

  ros::Rate rate2(1); // 1 Hz seems appropriate, for now.
  while(ros::ok()){
    if(callback_array.size() > 0){
      plan_array.insert(plan_array.end(), callback_array.begin(), callback_array.end());
      callback_array.clear();
    }
    if(plan_array.size() > 0){
      ros::Rate rate(10); // 1 Hz seems appropriate, for now.
      int length = plan_array.size();
      for(int i = 0; i < length; i++){
        if(m_executive->runPlan(plan_array[0].c_str())){
          if(firstPlan == true){
            firstPlan = false; 
          }
          int timeout = 0;
          // Times out after 3 seconds or the plan is registered as running.
          while(m_executive->getPlanState() && timeout < 30){
            ros::spinOnce();
            rate.sleep();
            timeout+=1;
            if(timeout % 10 == 0){
              ROS_ERROR("Plan not responding, timing out in %i seconds", (3 - timeout/10));
            }
          }
            if(timeout == 30){
              ROS_INFO ("Plan timed out, try again.");
  //            m_feedback.progress = "FAILED:" + plan_array[0];
    //          m_plannerActionServer.publishFeedback(m_feedback);
            }
            else{
      //        m_feedback.progress = "SUCCESS:" + plan_array[0];
        //      m_plannerActionServer.publishFeedback(m_feedback);
            }
        }
        else{
         // m_feedback.progress = "FAILED:" + plan_array[0];
         // m_plannerActionServer.publishFeedback(m_feedback);
        }

        plan_array.erase(plan_array.begin());
        while(!m_executive->getPlanState() && firstPlan == false){
          std::cout << "HERE" << std::endl;
          ros::spinOnce();
          rate2.sleep();
        }
        std::cout << "HERE2" << std::endl;
      //  m_feedback.progress = "COMPLETE";
       // m_plannerActionServer.publishFeedback(m_feedback);
      }
    }
    ros::spinOnce();
    rate2.sleep();
  }
}

void PlexilPlanner::executeCallback(const ow_plexil::PlannerGoalConstPtr &goal)
{
  
  if(goal->command.compare("ADD") == 0){
    plan_array.insert(plan_array.end(), goal->plans.begin(), goal->plans.end());
  }
  else if(goal->command.compare("RESET") == 0){
    plan_array.clear();
    ROS_INFO ("Plan list reset, current plan will finish execution before stopping");
  }
  else{
    ROS_ERROR("Command %s not recognized", goal->command.c_str());
  }

  m_result.result = true;
  m_plannerActionServer.setSucceeded(m_result);
}

