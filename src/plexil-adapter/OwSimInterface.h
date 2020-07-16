// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef OwSimInterface_H
#define OwSimInterface_H

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
