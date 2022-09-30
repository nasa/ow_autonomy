// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for
// Exploration Research and Simulation can be found in README.md in
// the root directory of this repository.

#ifndef Owlat_Adapter
#define Owlat_Adapter

// PLEXIL Interface adapter for OWLAT simulator.

#include "CommonAdapter.h"

class OwlatAdapter : public CommonAdapter
{
public:
  // No default constructor, only this specialized one.
  OwlatAdapter (PLEXIL::AdapterExecInterface&, const pugi::xml_node&);
  ~OwlatAdapter () = default;
  OwlatAdapter (const OwlatAdapter&) = delete;
  OwlatAdapter& operator= (const OwlatAdapter&) = delete;

  virtual bool initialize();
  virtual void lookupNow (const PLEXIL::State&, PLEXIL::StateCacheEntry&);
};

extern "C" {
  void initOwlatAdapter();
}

#endif
