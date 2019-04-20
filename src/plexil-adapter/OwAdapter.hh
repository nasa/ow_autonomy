#ifndef Ow_Adapter
#define Ow_Adapter

// PLEXIL Interface adapter.

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
  OwAdapter (const OwAdapter&);             // undefined, no copying
  OwAdapter& operator= (const OwAdapter&);  // undefined, no assignment

  bool isStateSubscribed(const State& state) const;
  std::set<State> m_subscribedStates;
};

extern "C" {
  void initOwAdapter();
}

#endif
