// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Adapter
#define Ow_Adapter

// PLEXIL interface adapter for OceanWATERS.

#include "LanderAdapter.h"

class OwInterface;

class OwAdapter : public LanderAdapter
{
public:
  // No default constructor, only this specialized one.
  OwAdapter (PLEXIL::AdapterExecInterface&, const pugi::xml_node&);
  ~OwAdapter () = default;
  OwAdapter (const OwAdapter&) = delete;
  OwAdapter& operator= (const OwAdapter&) = delete;

  virtual bool initialize();
  virtual void lookupNow (const PLEXIL::State&, PLEXIL::StateCacheEntry&);

 private:
  OwInterface* m_interface;
};

extern "C" {
  void initOwAdapter();
}

#endif
