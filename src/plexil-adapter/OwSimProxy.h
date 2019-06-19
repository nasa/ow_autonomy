#ifndef OwSimProxy_H
#define OwSimProxy_H

// A proxy for the OceanWorlds simulator.  Implements stubs for commands and
// telemetry.  Used for testing PLEXIL plans and helping to understand and
// define the required interface and functionality of the actual simulator.

// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

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

 private:
  OwSimProxy (const OwSimProxy&);             // undefined; no copying
  OwSimProxy& operator= (const OwSimProxy&);  // undefined; no assignment

};


#endif
