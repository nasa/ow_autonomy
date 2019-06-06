#include "OwSimProxy.hh"
#include <iostream>

using std::cerr;
using std::endl;
using std::string;

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

  STATE_STUB(TrenchLength, 10)
  else STATE_STUB(TrenchWidth, 10)
  else STATE_STUB(TrenchDepth, 2)
  else STATE_STUB(TrenchSlopeAngle, 30)
  else STATE_STUB(TrenchStart, 5)
  else STATE_STUB(TrenchIdentified, true)
  else STATE_STUB(TrenchTargetTimeout, 60)
  else STATE_STUB(ExcavationTimeout, 10)
  else STATE_STUB(ExcavationTimeout, 60)
  else STATE_STUB(SampleGood, true)
  else STATE_STUB(CollectAndTransferTimeout, 10)
  else retval = false;

  return retval;
}
