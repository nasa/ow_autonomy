// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Executive_H
#define Ow_Executive_H

// Autonomy Executive for OceanWATERS.

// The implementation embeds a PLEXIL executive and application.  Because only
// one PLEXIL executive can run in one process, this class is a singleton.

#include <string>
#include <ExecApplication.hh>
#include <InterfaceManager.hh>
#include <AdapterConfiguration.hh>

class OwExecutive
{
 public:
  static OwExecutive* instance();

  OwExecutive();
  OwExecutive (const OwExecutive&) = delete;
  OwExecutive& operator= (const OwExecutive&) = delete;
  ~OwExecutive() = default;

  bool initialize (const std::string& config_file);
  bool allPlansFinished(); // See warning in implementation.
  bool runPlan (const std::string& filename);
  PLEXIL::InterfaceManager* plexilInterfaceMgr();
  PLEXIL::AdapterConfiguration* plexilAdapterConfig();

 private:
  bool initializePlexilInterfaces (const std::string& config_file);
  PLEXIL::ExecApplication* m_plexil_app ;
};

#endif
