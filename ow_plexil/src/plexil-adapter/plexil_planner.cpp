#include "plexil_planner.h"
#include <string>
#include <ow_plexil/PlannerAction.h>
#include <actionlib/server/simple_action_server.h>
#include "OwExecutive.h"
#include "OwInterface.h"
#include <std_msgs/String.h>

PlexilPlanner::PlexilPlanner(){}

void PlexilPlanner::initialize()
{
  ROS_INFO("Starting planning node...");
  m_executive.reset(OwExecutive::instance());
  if (! m_executive->initialize()) {
    ROS_ERROR("Could not initialize OW executive, shutting down.");
  }
  OwInterface::instance()->initialize();

  m_genericNodeHandle = std::make_unique<ros::NodeHandle>();

  // wait for the first proper clock message before running the plan
  ros::Rate warmup_rate(0.1);
  ros::Time begin = ros::Time::now();
  while (ros::Time::now() - begin == ros::Duration(0.0))
  {
    ros::spinOnce();
    warmup_rate.sleep();
  }

  firstPlan = true;
  
  m_plannerCommandSubscriber = std::make_unique<ros::Subscriber>
      (m_genericNodeHandle->
       subscribe("/plexil_gui_commands", 20,
                 &PlexilPlanner::plannerCommandsCallback, this));

  m_planStatusPublisher = std::make_unique<ros::Publisher>
      (m_genericNodeHandle->advertise<std_msgs::String>
       ("/plexil_gui_plan_status", 20));


  ROS_INFO("Planning node started, ready for PLEXIL plans.");
}


void PlexilPlanner::start()
{
  ros::Rate rate(10); // 1 Hz seems appropriate, for now.
  ros::Rate rate2(1); // 1 Hz seems appropriate, for now.
  std_msgs::String status;
  while(ros::ok()){
    if(callback_array.size() > 0){
      plan_array.insert(plan_array.end(), callback_array.begin(), callback_array.end());
      callback_array.clear();
    }
    if(plan_array.size() > 0){
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
              status.data = "FAILED:" + plan_array[0];
              m_planStatusPublisher->publish(status);
            }
            else{
                status.data = "SUCCESS:" + plan_array[0];
                m_planStatusPublisher->publish(status);
            }
        }
        else{
            status.data = "FAILED:" + plan_array[0];
            m_planStatusPublisher->publish(status);
        }

        plan_array.erase(plan_array.begin());
        while(!m_executive->getPlanState() && firstPlan == false){
          ros::spinOnce();
          rate.sleep();
        }
          status.data = "COMPLETE";
          m_planStatusPublisher->publish(status);
      }
    }
    ros::spinOnce();
    rate2.sleep();
  }
}

void PlexilPlanner::plannerCommandsCallback(const ow_plexil::PlannerCommand::ConstPtr &msg)
{
  
  if(msg->command.compare("ADD") == 0){
    plan_array.insert(plan_array.end(), msg->plans.begin(), msg->plans.end());
  }
  else if(msg->command.compare("RESET") == 0){
    plan_array.clear();
    ROS_INFO ("Plan list reset, current plan will finish execution before stopping");
  }
  else{
    ROS_ERROR("Command %s not recognized", msg->command.c_str());
  }
}

