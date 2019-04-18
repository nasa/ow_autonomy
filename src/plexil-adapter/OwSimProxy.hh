#ifndef OwSimProxy_H
#define OwSimProxy_H

// A proxy for the OceanWorlds simulator.  Implements stubs for commands and
// telemetry.  Used for testing PLEXIL plans and helping to understand and
// define the required interface and functionality of the actual simulator.

#include <string>
#include <vector>
#include "Value.hh"

class OwSimProxy
{
 public:
  OwSimProxy () { }
  // Using compiler's destructor.
  bool lookup (const std::string& state_name,
               const std::vector<PLEXIL::Value>& args,
               PLEXIL::Value& value_out);
  bool DigTrench (float loc[3], float depth, float length, float width,
		  float pitch, float yaw, float dump[3]);


 private:
  OwSimProxy (const OwSimProxy&);             // undefined; no copying
  OwSimProxy& operator= (const OwSimProxy&);  // undefined; no assignment

};


#endif
