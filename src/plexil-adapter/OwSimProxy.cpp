// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "OwSimProxy.h"
#include "OwInterface.h"
#include <ros/ros.h>
#include <iostream>

using std::string;

static void stubbed_lookup (const string& name, const string& value)
{
  // This warning is annoying.  Could parameterize it.
  //  ROS_WARN("PLEXIL Adapter: Stubbed lookup of %s returning %s",
  //           name.c_str(), value.c_str());
}


// NOTE: This macro, and the stub it implements, are temporary.
#define STATE_STUB(name,val)                    \
  if (state_name == #name) {                    \
    stubbed_lookup (#name, #val);               \
    value_out = val;                            \
  }

bool OwSimProxy::lookup (const std::string& state_name,
                         const std::vector<PLEXIL::Value>& args,
                         PLEXIL::Value& value_out)
{
  bool retval = true;

  // Eventually, these stubs will go away.

  STATE_STUB(TrenchLength, 10)
  else STATE_STUB(TrenchGroundPosition, -0.155)
  else STATE_STUB(TrenchWidth, 10)
  else STATE_STUB(TrenchDepth, 2)
  else STATE_STUB(TrenchPitch, 0)
  else STATE_STUB(TrenchYaw, 0)
  else STATE_STUB(TrenchStartX, 5)
  else STATE_STUB(TrenchStartY, 10)
  else STATE_STUB(TrenchStartZ, 0)
  else STATE_STUB(TrenchDumpX, 0)
  else STATE_STUB(TrenchDumpY, 0)
  else STATE_STUB(TrenchDumpZ, 5)
  else STATE_STUB(TrenchIdentified, true)
  else STATE_STUB(TrenchTargetTimeout, 60)
  else STATE_STUB(ExcavationTimeout, 10)
  else STATE_STUB(ExcavationTimeout, 60)
  else STATE_STUB(SampleGood, true)
  else STATE_STUB(CollectAndTransferTimeout, 10)
  else STATE_STUB(VertFOV, 10) // should be 15
  else STATE_STUB(HorizFOV, 10) // should be 21
  else STATE_STUB(TorqueHighLimit, 800)

  else if (state_name == "TiltDegrees") {
    value_out = OwInterface::instance()->getTilt();
  }
  else if (state_name == "PanDegrees") {
    value_out = OwInterface::instance()->getPanDegrees();
  }
  else if (state_name == "PanVelocity") {
    value_out = OwInterface::instance()->getPanVelocity();
  }
  else if (state_name == "TiltVelocity") {
    value_out = OwInterface::instance()->getTiltVelocity();
  }
  else if (state_name == "HardTorqueLimitReached") {
    string s;
    args[0].getValue(s);
    value_out = OwInterface::instance()->hardTorqueLimitReached(s);
  }
  else if (state_name == "SoftTorqueLimitReached") {
    string s;
    args[0].getValue(s);
    value_out = OwInterface::instance()->softTorqueLimitReached(s);
  }
  else if (state_name == "Running") {
    string operation;
    args[0].getValue(operation);
    value_out = OwInterface::instance()->running (operation);
  }
  else if (state_name == "StateOfCharge") {
    value_out = OwInterface::instance()->getStateOfCharge();
  }
  else if (state_name == "RemainingUsefulLife") {
    value_out = OwInterface::instance()->getRemainingUsefulLife();
  }
  else if (state_name == "BatteryTemperature") {
    value_out = OwInterface::instance()->getBatteryTemperature();
  }
  else if (state_name == "GroundFound") {
    value_out = OwInterface::instance()->groundFound();
  }
  else if (state_name == "GroundPosition") {
    value_out = OwInterface::instance()->groundPosition();
  }
  // Faults
  else if (state_name == "SystemFault") {
    value_out = OwInterface::instance()->systemFault();
  }
  else if (state_name == "AntennaFault") {
    value_out = OwInterface::instance()->antennaFault();
  }
  else if (state_name == "ArmFault") {
    value_out = OwInterface::instance()->armFault();
  }
  else if (state_name == "PowerFault") {
    value_out = OwInterface::instance()->powerFault();
  }
  else retval = false;

  return retval;
}
