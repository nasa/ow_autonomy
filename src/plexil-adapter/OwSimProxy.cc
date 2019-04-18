#include "OwSimProxy.hh"
#include <iostream>

using std::cerr;
using std::cout;
using std::endl;
using std::string;
//using namespace PLEXIL;

static void stubbed_lookup (const string& name, const string& value)
{
  cerr << "WARNING: Stubbed lookup (returning " << value << "): " << name
       << endl;
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

  float trench_start[3] = {1,2,0};
  float trench_dump[3]  = {2,3,1};

  STATE_STUB(TrenchLength, 10)
  else STATE_STUB(TrenchWidth, 10)
  else STATE_STUB(TrenchDepth, 2)
  else STATE_STUB(TrenchYaw, 2)
  else STATE_STUB(TrenchPitch, 2)
  else STATE_STUB(TrenchSlopeAngle, 30)
  else STATE_STUB(TrenchStart, trench_start)
  else STATE_STUB(TrenchDump, trench_dump)
  else STATE_STUB(TrenchIdentified, true)
  else STATE_STUB(TrenchTargetTimeout, 60)
  else STATE_STUB(ExcavationTimeout, 10)
  else STATE_STUB(ExcavationTimeout, 60)
  else STATE_STUB(SampleGood, true)
  else STATE_STUB(CollectAndTransferTimeout, 10)
  else retval = false;

  return retval;
}

bool OwSimProxy::DigTrench (float loc[3], float depth, float length, float width,
			    float pitch, float yaw, float dump[3])
{
  cout << "OwSimProxy::DigTrench called" << endl;
  return true;
}
