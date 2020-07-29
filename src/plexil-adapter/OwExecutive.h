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
  static OwExecutive* instance();
  OwExecutive() { }
  ~OwExecutive();

  bool initialize ();
  bool runPlan (const std::string& filename);

 private:
  OwExecutive (const OwExecutive&);            // undefined (singleton)
  OwExecutive& operator= (const OwExecutive&); // undefined (singleton)
  static OwExecutive* m_instance;
};

#endif
