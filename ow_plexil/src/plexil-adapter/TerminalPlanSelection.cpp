// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "TerminalPlanSelection.h"
#include <string>

void TerminalPlanSelection::initialize()
{
  //create nodehandle
  m_genericNodeHandle = std::make_unique<ros::NodeHandle>();
  plan_running = false;
  //initialize subscriber and publisher
  m_planSelectionStatusSubscriber = std::make_unique<ros::Subscriber>
      (m_genericNodeHandle->
       subscribe("/plexil_plan_selection_status", 20,
                 &TerminalPlanSelection::planSelectionStatusCallback, this));

  m_planSelectionCommandPublisher = std::make_unique<ros::Publisher>
      (m_genericNodeHandle->advertise<ow_plexil::PlanSelectionCommand>
       ("/plexil_plan_selection_commands", 20));

  //wait for subscriber
  ros::Rate rate(10); // 10 Hz seems appropriate, for now.
  while(ros::ok() && m_planSelectionCommandPublisher->getNumSubscribers() == 0){
    ros::spinOnce();
    rate.sleep();
  }
}

void TerminalPlanSelection::start(bool initial_plan)
{
  if(initial_plan == true){
    plan_running = true;
  }
  std::vector<std::string> plan_array;

  ros::Rate rate(10); // 10 Hz seems appropriate, for now.
  std::string input; 
  // loops until terminated by user, prompts them to enter any additional plans after the
  // previous plan has been run to completion.
  while (ros::ok()) {
    if(plan_running == false){ // checks to see if previous plan finished
      std::cout << "\nEnter any additional plan to be run (or use the GUI): " << std::endl;
      std::getline(std::cin, input); 
      // if input is not empty we send the plan to the plan selection node for execution
      if(input != ""){
        ow_plexil::PlanSelectionCommand instruction;
        instruction.command = "ADD";
        plan_array.push_back(input);
        instruction.plans = plan_array;
        plan_running = true;
        m_planSelectionCommandPublisher->publish(instruction);
        plan_array.clear();
      }
    }
    else{
      ros::spinOnce();
      rate.sleep();
    }
  }
}
  
void TerminalPlanSelection::planSelectionStatusCallback(const std_msgs::String::ConstPtr& msg)
{
  // if plan is set as complete or failed we know that no plan is currently running
  if(msg->data.compare("COMPLETE") == 0 || msg->data.find("FAILED") != std::string::npos){
    plan_running = false; 
  }
}
