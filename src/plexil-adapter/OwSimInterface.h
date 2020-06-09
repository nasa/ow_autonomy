#ifndef OwSimInterface_H
#define OwSimInterface_H

// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.

// Implementation-independent interface to the OceanWATERS simulator.

#include <string>
#include <vector>
#include "Value.hh"

class OwSimInterface
{
 public:
  OwSimInterface () = default;
  ~OwSimInterface () = default;
  OwSimInterface (const OwSimInterface&) = delete;
  OwSimInterface& operator= (const OwSimInterface&) = delete;

  bool lookup (const std::string& state_name,
               const std::vector<PLEXIL::Value>& args,
               PLEXIL::Value& value_out);
  bool DigTrench (float start_x, float start_y, float start_z,
                  float depth, float length, float width, float pitch, float yaw,
                  float dump_x, float dump_y, float dump_z);
};


#endif
