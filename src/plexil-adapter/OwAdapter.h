#ifndef Ow_Adapter
#define Ow_Adapter

// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.

// PLEXIL Interface adapter for OceanWATERS.

// PLEXIL API
#include "Command.hh"
#include "InterfaceAdapter.hh"
#include "Value.hh"

using namespace PLEXIL;

class OwAdapter : public InterfaceAdapter
{
public:
  // No default constructor, only this specialized one.
  OwAdapter (AdapterExecInterface&, const pugi::xml_node&);
  ~OwAdapter ();
  OwAdapter (const OwAdapter&) = delete;
  OwAdapter& operator= (const OwAdapter&) = delete;

  virtual bool initialize();
  virtual bool start();
  virtual bool stop();
  virtual bool reset();
  virtual bool shutdown();
  virtual void invokeAbort(Command *cmd);

  virtual void executeCommand(Command *cmd);
  virtual void lookupNow (State const& state, StateCacheEntry &entry);
  virtual void subscribe(const State& state);
  virtual void unsubscribe(const State& state);
  void propagateValueChange (const State&, const std::vector<Value>&) const;

private:
  bool isStateSubscribed(const State& state) const;
  std::set<State> m_subscribedStates;
};

extern "C" {
  void initOwAdapter();
}

#endif
