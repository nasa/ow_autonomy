// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "TerminalPlanSelection.h"
#include <ow_plexil/PlanSelection.h>
#include <string>

void TerminalPlanSelection::initialize()
{
  //create nodehandle
  m_genericNodeHandle = std::make_unique<ros::NodeHandle>();
  m_plan_running = false;

  //wait for server
  ros::Rate rate(10); // 10 Hz seems appropriate, for now.
  while(ros::ok() &&
        ros::service::exists("/plexil_plan_selection", false) == false) {
    ros::spinOnce();
    rate.sleep();
  }

  m_planSelectionServiceClient = std::make_unique<ros::ServiceClient>
    (m_genericNodeHandle->serviceClient<ow_plexil::PlanSelection>
     ("/plexil_plan_selection", this));

  //initialize subscriber
  m_planSelectionStatusSubscriber = std::make_unique<ros::Subscriber>
    (m_genericNodeHandle->
     subscribe("/plexil_plan_selection_status", 20,
               &TerminalPlanSelection::planSelectionStatusCallback, this));
}

void TerminalPlanSelection::start(bool initial_plan)
{
  if(initial_plan == true){
    m_plan_running = true;
  }
  std::vector<std::string> plan_array;

  ros::Rate rate(10); // 10 Hz seems appropriate, for now.
  std::string input;
  // loops until terminated by user, prompts them to enter any
  // additional plans after the previous plan has been run to
  // completion.
  while (ros::ok()) {
    if (m_plan_running == false) {
      // checks to see if previous plan finished
      std::cout << "\nEnter any additional plan to be run (or use the GUI): "
                << std::endl;
      std::cin >> input;
      std::cin.ignore();
      // if input is not empty we send the plan to the plan selection
      // node for execution
      if(input != "") {
        ow_plexil::PlanSelection instruction;
        instruction.request.command = "ADD";
        plan_array.push_back(input);
        instruction.request.plans = plan_array;
        if(m_planSelectionServiceClient->call(instruction)){
          m_plan_running = true;
          plan_array.clear();
        }
        else {
          ROS_ERROR("Unable to contact plan_selection_server...");
        }
      }
    }
    else {
      ros::spinOnce();
      rate.sleep();
    }
  }
}

void TerminalPlanSelection::planSelectionStatusCallback
(const std_msgs::String::ConstPtr& msg)
{
  // if plan is set as complete or failed we know that no plan is
  // currently running
  if(msg->data.compare("COMPLETE") == 0 ||
     msg->data.find("FAILED") != std::string::npos) {
    m_plan_running = false;
  }
}
