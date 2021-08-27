// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Terminal_Plan_Selection_H
#define Terminal_Plan_Selection_H

#include <ros/ros.h>
#include <std_msgs/String.h>

class TerminalPlanSelection{
  public:
    TerminalPlanSelection() = default;
    ~TerminalPlanSelection() = default;
    void initialize();
    void start(bool initial_plan);

  private:
    TerminalPlanSelection(const TerminalPlanSelection&) = delete;
    TerminalPlanSelection& operator = (const TerminalPlanSelection&) = delete;
    void planSelectionStatusCallback(const std_msgs::String::ConstPtr &msg);
    
    std::unique_ptr<ros::NodeHandle> m_genericNodeHandle;
    std::unique_ptr<ros::Subscriber> m_planSelectionStatusSubscriber;
    std::unique_ptr<ros::ServiceClient> m_planSelectionServiceClient;
    bool m_plan_running;
};

#endif

