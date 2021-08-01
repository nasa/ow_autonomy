// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Adapter
#define Ow_Adapter

// PLEXIL Interface adapter for OceanWATERS.

#include "CommonAdapter.h"

// PLEXIL
#include <Command.hh>
#include <Value.hh>

class OwAdapter : public CommonAdapter
{
public:
  // No default constructor, only this specialized one.
  OwAdapter (AdapterExecInterface&, const pugi::xml_node&);
  ~OwAdapter ();
  OwAdapter (const OwAdapter&) = delete;
  OwAdapter& operator= (const OwAdapter&) = delete;

  virtual bool initialize();
  virtual void lookupNow (const State&, StateCacheEntry&);
};

extern "C" {
  void initOwAdapter();
}

#endif
