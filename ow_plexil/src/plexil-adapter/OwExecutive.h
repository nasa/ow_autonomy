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
  
  // Deletes the singleton instance.  Would need to be explicitly called in the
  // present singleton scheme, since the singleton instance pointer is private
  // and only accessible through instance().  This function is idempotent, and
  // effectively useless in the current context, because the containing process
  // can only be terminated externally, e.g. with a keyboard interrupt.
  ~OwExecutive();

  // Singleton instance constructor and accessor.
  static OwExecutive* instance();

  bool initialize (const std::string& config_file);
  bool getPlanState(); // returns true if current plan is finished executing
  bool runPlan (const std::string& filename);

 private:
  OwExecutive();
  OwExecutive (const OwExecutive&) = delete;
  OwExecutive& operator= (const OwExecutive&) = delete;

  static OwExecutive* m_instance;
};

#endif
