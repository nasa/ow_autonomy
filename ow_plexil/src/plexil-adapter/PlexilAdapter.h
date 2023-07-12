// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Plexil_Adapter
#define Plexil_Adapter

// PLEXIL Interface adapter base class to be specialized for each testbed.

// PLEXIL
#include <InterfaceAdapter.hh>
#include <Command.hh>
#include <Value.hh>
#include <LookupHandler.hh>

// C++
#include <set>
#include <map>
#include <vector>

class PlexilAdapter : public PLEXIL::InterfaceAdapter
{
public:
  PlexilAdapter() = default;
  virtual ~PlexilAdapter() = default;
  PlexilAdapter (const PlexilAdapter&) = delete;
  PlexilAdapter& operator= (const PlexilAdapter&) = delete;

  virtual bool initialize();
  virtual bool start();
  virtual void stop();
  //  virtual bool reset();
  //  virtual bool shutdown();
  virtual void invokeAbort(PLEXIL::Command *cmd);
  virtual void subscribe(const PLEXIL::State& state);
  virtual void unsubscribe(const PLEXIL::State& state);
  virtual void lookupNow (const PLEXIL::State&, PLEXIL::LookupReceiver&) { }
  void propagateValueChange (const PLEXIL::State&,
                             const std::vector<PLEXIL::Value>&);

protected:
  PlexilAdapter (PLEXIL::AdapterExecInterface&, const pugi::xml_node&);
  bool isStateSubscribed (const PLEXIL::State& state) const;
  std::set<PLEXIL::State> m_subscribedStates;
};

#endif
