// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Lander_Adapter
#define Lander_Adapter

// PLEXIL adapter base class for OceanWATERS and JPL's OWLAT

#include "PlexilAdapter.h"

class LanderAdapter : public PlexilAdapter
{
public:
  // No default constructor, only this specialized one.
  LanderAdapter (PLEXIL::AdapterExecInterface&, const pugi::xml_node&);
  virtual ~LanderAdapter () = 0 = default;
  LanderAdapter (const LanderAdapter&) = delete;
  LanderAdapter& operator= (const LanderAdapter&) = delete;

  virtual bool initialize();
  virtual void lookupNow (const PLEXIL::State&, PLEXIL::StateCacheEntry&);
};

extern "C" {
  void initLanderAdapter();
}

#endif
