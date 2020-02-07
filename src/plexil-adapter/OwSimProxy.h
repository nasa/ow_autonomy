#ifndef OwSimProxy_H
#define OwSimProxy_H

// Originally, a *proxy* for the OceanWATERS (OW) simulator, providing stubs for
// commands and telemetry.  Now, in addition, a *wrapper* for state queries to
// the OW simulator.  Eventually it will be just the wrapper, and at this point,
// this class may be refactored or go away.

// __BEGIN_LICENSE__
// Copyright (c) 2018-2020, United States Government as represented by the
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
