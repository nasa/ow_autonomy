// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

#include "OwSimProxy.h"
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

  // TODO: streamline this, and make it late-bindable.

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
  else STATE_STUB(VertFOV, 15)
  else STATE_STUB(HorizFOV, 21)
  else retval = false;

  return retval;
}
