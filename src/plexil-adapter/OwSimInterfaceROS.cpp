// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// ROS-based implementation of Sim interface.

#include "OwSimInterface.h"
#include <iostream>
#include <ros/ros.h>

using std::cerr;
using std::cout;
using std::endl;
using std::string;


static void stubbedLookup (const string& name, const string& value)
{
  cerr << "WARNING: Stubbed lookup (returning " << value << "): " << name
       << endl;
}


// NOTE: This macro, and the stub it implements, are temporary.
#define STATE_STUB(name,val)                    \
  if (state_name == #name) {                    \
    stubbedLookup (#name, #val);               \
    value_out = val;                            \
  }

bool OwSimInterface::lookup (const std::string& state_name,
                             const std::vector<PLEXIL::Value>& args,
                             PLEXIL::Value& value_out)
{
  bool retval = true;

  // TODO: streamline this, and make it late-bindable.

  STATE_STUB(TrenchLength, 10)
  else STATE_STUB(TrenchWidth, 10)
  else STATE_STUB(TrenchDepth, 2)
  else STATE_STUB(TrenchYaw, 2)
  else STATE_STUB(TrenchPitch, 2)
  else STATE_STUB(TrenchSlopeAngle, 30)
  else STATE_STUB(TrenchStartX, 2)
  else STATE_STUB(TrenchStartY, 3)
  else STATE_STUB(TrenchStartZ, 1)
  else STATE_STUB(TrenchDumpX, 0)
  else STATE_STUB(TrenchDumpY, 0)
  else STATE_STUB(TrenchDumpZ, 1)
  else STATE_STUB(TrenchIdentified, true)
  else STATE_STUB(TrenchTargetTimeout, 60)
  else STATE_STUB(ExcavationTimeout, 10)
  else STATE_STUB(ExcavationTimeout, 60)
  else STATE_STUB(SampleGood, true)
  else STATE_STUB(CollectAndTransferTimeout, 10)
  else retval = false;

  return retval;
}

bool OwSimInterface::DigTrench (float start_x, float start_y, float start_z,
                                float depth, float length, float width,
                                float pitch, float yaw,
                                float dump_x, float dump_y, float dump_z)
{
  cout << "OwSimInterface::DigTrench called" << endl;
  return true;
}
