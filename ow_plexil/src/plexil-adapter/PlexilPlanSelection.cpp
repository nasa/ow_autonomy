// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.


#include "PlexilPlanSelection.h"
#include <string>
#include <std_msgs/String.h>

void PlexilPlanSelection::initialize(std::string initial_plan)
{
  ROS_INFO("Starting plexil node...");
  //initialzing the Executive
  m_executive.reset(OwExecutive::instance());
  if (! m_executive->initialize()) {
    ROS_ERROR("Could not initialize OW executive, shutting down.");
  }

  //initialize the OwInterface
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

  // if launch argument plan is given we add it
  if(initial_plan.compare("None") != 0) {
    plan_array.push_back(initial_plan);
  }
  //workaround for the getPlanState not returning correctly for first plan
  m_first_plan = true;
  
  //initialize subscriber and publisher
  m_planSelectionCommandSubscriber = std::make_unique<ros::Subscriber>
      (m_genericNodeHandle->
       subscribe("/plexil_plan_selection_commands", 20,
                 &PlexilPlanSelection::planSelectionCommandsCallback, this));
  m_planSelectionStatusPublisher = std::make_unique<ros::Publisher>
      (m_genericNodeHandle->advertise<std_msgs::String>
       ("/plexil_plan_selection_status", 20));

  ROS_INFO("Plexil node started, ready for PLEXIL plans.");
}


void PlexilPlanSelection::start()
{
  ros::Rate rate(10); // 10 Hz for overall rate we are spinning
  ros::Rate rate2(1); // 1 Hz for timeout sleep
  std_msgs::String status;
  while(ros::ok()){
    //if we have plans in our plan array we begin the control loop
    if(plan_array.size() > 0){
      int length = plan_array.size();
      for(int i = 0; i < length; i++){
        //try to run the plan
        if(m_executive->runPlan(plan_array[0].c_str())){
          //workaround for getPlanState not working on first plan
          if(m_first_plan == true){
            m_first_plan = false; 
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
            //if timed out we set plan as failed for GUI
            if(timeout == 30){
              ROS_INFO ("Plan timed out, try again.");
              status.data = "FAILED:" + plan_array[0];
              m_planSelectionStatusPublisher->publish(status);
            }
            //otherwise we set it as running
            else{
                status.data = "SUCCESS:" + plan_array[0];
                m_planSelectionStatusPublisher->publish(status);
            }
        }
        //if error from run() we set as failed for GUI
        else{
            status.data = "FAILED:" + plan_array[0];
            m_planSelectionStatusPublisher->publish(status);
        }
        //delete the plan we just ran from plan array
        plan_array.erase(plan_array.begin());
        //wait for current plan to finish before running next plan
        while(!m_executive->getPlanState() && m_first_plan == false){
          ros::spinOnce();
          rate.sleep();
        }
        //Once plan is finished set status to complete for GUI
        status.data = "COMPLETE";
        m_planSelectionStatusPublisher->publish(status);
        //checks if plan_array has been cleared
        if(plan_array.size() == 0){
          break; 
        }
      }
    }
    //if no plans we spinonce and sleep before checking again 
    ros::spinOnce();
    rate2.sleep();
  }
}

void PlexilPlanSelection::planSelectionCommandsCallback(const ow_plexil::PlanSelectionCommand::ConstPtr &msg)
{
  //if command is ADD we add given plans to the plan_array 
  if(msg->command.compare("ADD") == 0){
    plan_array.insert(plan_array.end(), msg->plans.begin(), msg->plans.end());
  }
  //if command is RESET  delete all plans in the plan_array 
  else if(msg->command.compare("RESET") == 0){
    plan_array.clear();
    ROS_INFO ("Plan list cleared, current plan will finish execution before stopping");
  }
  else{
    ROS_ERROR("Command %s not recognized", msg->command.c_str());
  }
}

