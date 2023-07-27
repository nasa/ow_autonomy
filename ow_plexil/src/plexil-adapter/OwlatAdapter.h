// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
// Exploration Research and Simulation can be found in README.md in
// the root directory of this repository.

#ifndef Owlat_Adapter
#define Owlat_Adapter

// PLEXIL Interface adapter for OWLAT simulator.

#include "LanderAdapter.h"

class OwlatInterface;

class OwlatAdapter : public LanderAdapter
{
public:
  OwlatAdapter (PLEXIL::AdapterExecInterface&, PLEXIL::AdapterConf*);
  OwlatAdapter () = delete;
  ~OwlatAdapter () = default;
  OwlatAdapter (const OwlatAdapter&) = delete;
  OwlatAdapter& operator= (const OwlatAdapter&) = delete;

  virtual bool initialize (PLEXIL::AdapterConfiguration*) final;
};

extern "C" {
  void initOwlatAdapter();
}

#endif
