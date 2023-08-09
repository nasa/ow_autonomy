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
#include <set>
#include <vector>

class PlexilAdapter : public PLEXIL::InterfaceAdapter
{
public:
  PlexilAdapter() = delete;
  virtual ~PlexilAdapter() = default;
  PlexilAdapter (const PlexilAdapter&) = delete;
  PlexilAdapter& operator= (const PlexilAdapter&) = delete;

  virtual bool initialize(PLEXIL::AdapterConfiguration*) override;
  virtual bool start() override;
  virtual void stop() override;
  virtual void subscribe(const PLEXIL::State& state); // obsolete?
  virtual void unsubscribe(const PLEXIL::State& state);  // obsolete?
  void propagateValueChange (const PLEXIL::State&,
                             const std::vector<PLEXIL::Value>&);

protected:
  PlexilAdapter (PLEXIL::AdapterExecInterface&, PLEXIL::AdapterConf*);
  bool isStateSubscribed (const PLEXIL::State& state) const;
  std::set<PLEXIL::State> m_subscribedStates;
};

#endif
