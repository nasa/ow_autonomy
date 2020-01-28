// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

#include "OwSimProxy.h"
#include "OwInterface.h"
#include <ros/ros.h>
#include <iostream>

using std::string;

static void stubbed_lookup (const string& name, const string& value)
{
  ROS_WARN("PLEXIL Adapter: Stubbed lookup of %s returning %s",
           name.c_str(), value.c_str());
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
  else STATE_STUB(TrenchWidth, 10)
  else STATE_STUB(TrenchDepth, 2)
  else STATE_STUB(TrenchPitch, 0)
  else STATE_STUB(TrenchYaw, 0)
  else STATE_STUB(TrenchSlopeAngle, 30)
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

  else if (state_name == "TiltDegrees") {
    value_out = OwInterface::instance()->getTilt();
  }
  else if (state_name == "PanDegrees") {
    value_out = OwInterface::instance()->getPanDegrees();
  }
  else if (state_name == "PanVelocity") {
    value_out = OwInterface::instance()->getPanVelocity();
  }


  else retval = false;

  return retval;
}
