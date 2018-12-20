#ifndef Ow_Adapter
#define Ow_Adapter

// PLEXIL Interface adapter for Ocean WATERS.

// PLEXIL API
#include "Command.hh"
#include "InterfaceAdapter.hh"
#include "Value.hh"

//using namespace PLEXIL;

class OwAdapter : public InterfaceAdapter
{
public:
  OwAdapter (AdapterExecInterface&, const pugi::xml_node&);

  // using compiler's default copy constructor, assignment, destructor
  
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
  Value fetch (const std::string& state_name, const std::vector<Value>& args);

  bool isStateSubscribed(const State& state) const;
  std::set<State> m_subscribedStates;
};

extern "C" {
  void initOwAdapter();
}

#endif
