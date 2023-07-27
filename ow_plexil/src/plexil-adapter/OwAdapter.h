// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Adapter
#define Ow_Adapter

// PLEXIL interface adapter for OceanWATERS.

#include "LanderAdapter.h"

class OwAdapter : public LanderAdapter
{
public:
  OwAdapter (PLEXIL::AdapterExecInterface&, PLEXIL::AdapterConf*);
  OwAdapter () = delete;
  ~OwAdapter () = default;
  OwAdapter (const OwAdapter&) = delete;
  OwAdapter& operator= (const OwAdapter&) = delete;

  virtual bool initialize (PLEXIL::AdapterConfiguration*) final;
  virtual void lookupNow (const PLEXIL::State&, PLEXIL::LookupReceiver*);
};

extern "C" {
  void initOwAdapter();
}

#endif
