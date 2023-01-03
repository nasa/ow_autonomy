// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "PlexilPlanSelection.h"
#include "OwExecutive.h"
#include <string>
#include <std_msgs/String.h>

void PlexilPlanSelection::initialize(std::string initial_plan)
{
  ROS_INFO("Starting PLEXIL executive node...");
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

  //initialize service
  m_planSelectionService = std::make_unique<ros::ServiceServer>
      (m_genericNodeHandle->
       advertiseService("/plexil_plan_selection",
       &PlexilPlanSelection::planSelectionServiceCallback, this));

  //initialize publisher
  m_planSelectionStatusPublisher = std::make_unique<ros::Publisher>
      (m_genericNodeHandle->advertise<std_msgs::String>
       ("/plexil_plan_selection_status", 20));

  ROS_INFO("Executive node started, ready for PLEXIL plans.");
}

void PlexilPlanSelection::start()
{
  ros::Rate rate(10); // 10 Hz for overall rate we are spinning
  int length = 0;
  while(ros::ok()){
    //if we have plans in our plan array we begin the control loop
    if(plan_array.size() > 0){
      length = plan_array.size();
      for(int i = 0; i < length; i++){
        //trys to run the current plan
        runCurrentPlan();
        //waits until plan finishes running
        waitForPlan();
        //checks if plan_array has been cleared
        if(plan_array.size() == 0){
          break;
        }
      }
    }
    //if no plans we spinonce and sleep before checking again
    ros::spinOnce();
    rate.sleep();
  }
}

void PlexilPlanSelection::waitForPlan(){
  std_msgs::String status;
  ros::Rate rate(10); // 10 Hz for overall rate we are spinning
  //wait for current plan to finish before running next plan
  while(!OwExecutive::instance()->getPlanState() && m_first_plan == false){
    ros::spinOnce();
    rate.sleep();
  }

  //Once plan is finished set status to complete for GUI
  status.data = "COMPLETE";
  m_planSelectionStatusPublisher->publish(status);
}

void PlexilPlanSelection::runCurrentPlan(){
  std_msgs::String status;
  ros::Rate rate(10); // 10 Hz for overall rate we are spinning
  //try to run the plan
  if(OwExecutive::instance()->runPlan(plan_array[0].c_str())){
    //workaround for getPlanState not working on first plan
    if(m_first_plan == true){
      m_first_plan = false;
    }
    // Times out after 3 seconds or the plan is registered as running.
    int timeout = 0;
    while(OwExecutive::instance()->getPlanState() && timeout < 30){
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
}

bool PlexilPlanSelection::planSelectionServiceCallback
(ow_plexil::PlanSelection::Request &req,
 ow_plexil::PlanSelection::Response &res)
{
  //if command is ADD we add given plans to the plan_array
  if(req.command.compare("ADD") == 0){
    plan_array.insert(plan_array.end(), req.plans.begin(), req.plans.end());
    res.success = true;
  }
  //if command is RESET  delete all plans in the plan_array
  else if(req.command.compare("RESET") == 0){
    plan_array.clear();
    ROS_INFO ("Plan list cleared, "
              "current plan will finish execution before stopping");
    res.success = true;
  }
  else{
    ROS_ERROR("Command %s not recognized", req.command.c_str());
    res.success = false;
  }
  return true;
}
