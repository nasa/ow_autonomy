// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Executive_H
#define Ow_Executive_H

// Autonomy Executive for OceanWATERS.

// The implementation embeds a PLEXIL executive and application.  Because only
// one PLEXIL executive can run in one process, this class is a singleton.

#include <string>

class OwExecutive
{
 public:
  OwExecutive() { }
  ~OwExecutive();
  OwExecutive (const OwExecutive&) = delete;
  OwExecutive& operator= (const OwExecutive&) = delete;
  
  static OwExecutive* instance();

  bool initialize ();
  bool runPlan (const std::string& filename);

 private:
  static OwExecutive* m_instance;
};

#endif
