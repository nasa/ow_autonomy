// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef OwSimProxy_H
#define OwSimProxy_H

// Originally, a *proxy* for the OceanWATERS (OW) simulator, providing stubs for
// commands and telemetry.  Now, in addition, a *wrapper* for state queries to
// the OW simulator.  Eventually it will be just the wrapper, and at this point,
// this class may be refactored or go away.

#include <string>
#include <vector>
#include "Value.hh"

class OwSimProxy
{
 public:
  OwSimProxy () = default;
  ~OwSimProxy () = default;
  OwSimProxy (const OwSimProxy&) = delete;
  OwSimProxy& operator= (const OwSimProxy&) = delete;

  bool lookup (const std::string& state_name,
               const std::vector<PLEXIL::Value>& args,
               PLEXIL::Value& value_out);
};

#endif
