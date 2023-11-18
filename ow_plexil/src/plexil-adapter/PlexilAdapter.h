// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Plexil_Adapter
#define Plexil_Adapter

// PLEXIL Interface adapter base class to be specialized for each testbed.

// PLEXIL
#include <InterfaceAdapter.hh>
#include <AdapterExecInterface.hh>
#include <AdapterConfiguration.hh>
#include <Value.hh>

// C++
#include <vector>

class PlexilAdapter : public PLEXIL::InterfaceAdapter
{
public:
  PlexilAdapter() = delete;
  virtual ~PlexilAdapter() = 0;
  PlexilAdapter (const PlexilAdapter&) = delete;
  PlexilAdapter& operator= (const PlexilAdapter&) = delete;

  virtual bool initialize(PLEXIL::AdapterConfiguration*) override;
  virtual bool start() override;
  virtual void stop() override;
  void propagateValueChange (const PLEXIL::State&,
                             const std::vector<PLEXIL::Value>&);

protected:
  PlexilAdapter (PLEXIL::AdapterExecInterface&, PLEXIL::AdapterConf*);
};

#endif
