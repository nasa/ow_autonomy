#ifndef OwSimInterface_H
#define OwSimInterface_H

// Implementation-independent interface to the OceanWorlds simulator.

// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

#include <string>
#include <vector>
#include "Value.hh"

class OwSimInterface
{
 public:
  OwSimInterface () { }
  // Using compiler's destructor.
  bool lookup (const std::string& state_name,
               const std::vector<PLEXIL::Value>& args,
               PLEXIL::Value& value_out);
  bool DigTrench (float start_x, float start_y, float start_z,
                  float depth, float length, float width, float pitch, float yaw,
                  float dump_x, float dump_y, float dump_z);


 private:
  OwSimInterface (const OwSimInterface&);             // undefined; no copying
  OwSimInterface& operator= (const OwSimInterface&);  // undefined; no assignment

};


#endif
