// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Plexil_Planner_H
#define Plexil_Planner_H

#include <ros/ros.h>
#include <ow_plexil/PlanSelection.h>

class PlexilPlanSelection{
  public:
    PlexilPlanSelection() = default;
    ~PlexilPlanSelection() = default;
    void initialize(std::string initial_plan);
    void start();


  private:
    PlexilPlanSelection(const PlexilPlanSelection&) = delete;
    PlexilPlanSelection& operator = (const PlexilPlanSelection&) = delete;
    bool planSelectionServiceCallback(ow_plexil::PlanSelection::Request&,
                                      ow_plexil::PlanSelection::Response&);
    void runCurrentPlan();
    void waitForPlan();

    std::unique_ptr<ros::NodeHandle> m_genericNodeHandle;
    std::unique_ptr<ros::ServiceServer> m_planSelectionService;
    std::unique_ptr<ros::Publisher> m_planSelectionStatusPublisher;
    std::vector<std::string> plan_array;
    bool m_first_plan;
 
};

#endif

