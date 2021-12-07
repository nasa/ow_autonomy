// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Common_Adapter
#define Common_Adapter

// PLEXIL Interface adapter base class to be specialized for each testbed.

// PLEXIL API
#include "InterfaceAdapter.hh"
#include "Command.hh"
#include "Value.hh"

#include <set>
#include <map>
#include <vector>

class CommonAdapter : public PLEXIL::InterfaceAdapter
{
public:
  CommonAdapter() = delete;  // only a specialized constructor for subclasses
  virtual ~CommonAdapter() = default;
  CommonAdapter (const CommonAdapter&) = delete;
  CommonAdapter& operator= (const CommonAdapter&) = delete;

  virtual bool initialize();
  virtual bool start();
  virtual bool stop();
  virtual bool reset();
  virtual bool shutdown();
  virtual void invokeAbort(PLEXIL::Command *cmd);
  virtual void subscribe(const PLEXIL::State& state);
  virtual void unsubscribe(const PLEXIL::State& state);
  void propagateValueChange (const PLEXIL::State&,
                             const std::vector<PLEXIL::Value>&) const;

protected:
  CommonAdapter (PLEXIL::AdapterExecInterface&, const pugi::xml_node&);
  bool isStateSubscribed (const PLEXIL::State& state) const;
  std::set<PLEXIL::State> m_subscribedStates;
};

#endif
