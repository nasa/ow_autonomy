#include "OwAdapter.hh"
#include "subscriber.hh"

// PLEXIL API
#include <AdapterConfiguration.hh>
#include <AdapterFactory.hh>
#include <AdapterExecInterface.hh>
#include <ArrayImpl.hh>
#include <Debug.hh>
#include <Expression.hh>
#include <StateCacheEntry.hh>

// C++/C
#include <list>
//#include <sstream>
//#include <math.h>
using std::string;
using std::vector;
using std::list;
using std::copy;
using std::cerr;
using std::cout;
using std::endl;


///////////////////////////// Conveniences //////////////////////////////////

// A prettier name for the "unknown" value.
static Value const Unknown;

// An empty argument vector.
static vector<Value> const EmptyArgs;


static void owprint (const vector<Value>& args)
{
  std::ostringstream out;

  for (vector<Value>::const_iterator iter = args.begin();
       iter != args.end();
       iter++) out << *iter;
  cerr << out.str() << endl;
}

static void unimplemented (const string& name)
{
  cout << "Command " << name << " not yet implemented!" << endl;
}


////////////////////// Publish/subscribe support ////////////////////////////

// A localized handle on the adapter, which allows a
// decoupling between the sample system and adapter.
static OwAdapter* TheAdapter;

static State createState (const string& state_name, const vector<Value>& value)
{
  State state(state_name, value.size());
  if (value.size() > 0)
  {
    for(size_t i=0; i<value.size();i++)
    {
      state.setParameter(i, value[i]);
    }
  }
  return state;
}

static void propagate (const State& state, const vector<Value>& value)
{
  TheAdapter->propagateValueChange (state, value);
}


static void receiveBool (const string& state_name, bool val)
{
  propagate (createState(state_name, EmptyArgs),
             vector<Value> (1, val));
}

static void receiveString (const string& state_name, const string& val)
{
  propagate (createState(state_name, EmptyArgs),
             vector<Value> (1, val));
}

static void receiveBoolString (const string& state_name, bool val, const string& arg)
{
  propagate (createState(state_name, vector<Value> (1, arg)),
             vector<Value> (1, val));
}

void OwAdapter::propagateValueChange (const State& state,
                                       const vector<Value>& vals) const
{
  if (!isStateSubscribed(state)) return;

  m_execInterface.handleValueChange (state, vals.front());
  m_execInterface.notifyOfExternalEvent();
}

bool OwAdapter::isStateSubscribed(const State& state) const
{
  return m_subscribedStates.find(state) != m_subscribedStates.end();
}



///////////////////////////// Member functions //////////////////////////////////


OwAdapter::OwAdapter(AdapterExecInterface& execInterface,
                             const pugi::xml_node& configXml) :
  InterfaceAdapter(execInterface, configXml)
{
  debugMsg("OwAdapter", " created.");
}

bool OwAdapter::initialize()
{
  g_configuration->defaultRegisterAdapter(this);
  TheAdapter = this;
  setSubscriber (receiveBool);
  setSubscriber (receiveString);
  setSubscriber (receiveBoolString);

  debugMsg("OwAdapter", " initialized.");
  return true;
}

bool OwAdapter::start()
{
  debugMsg("OwAdapter", " started.");
  return true;
}

bool OwAdapter::stop()
{
  debugMsg("OwAdapter", " stopped.");
  return true;
}

bool OwAdapter::reset()
{
  debugMsg("OwAdapter", " reset.");
  return true;
}

bool OwAdapter::shutdown()
{
  debugMsg("OwAdapter", " shut down.");
  return true;
}

void OwAdapter::invokeAbort(Command *cmd)
{
}

static void stubbed_lookup (const string& name, const string& value)
{
  cerr << "WARNING: Stubbed lookup (returning " << value << "): " << name
       << endl;
}


// Sends a command (as invoked in a Plexil command node) to the system and sends
// the status, and return value if applicable, back to the executive.
//
void OwAdapter::executeCommand(Command *cmd)
{
  string name = cmd->getName();
  debugMsg("OwAdapter:executeCommand", " for " << name);

  // Arguments
  vector<Value> argv(12);
  const vector<Value>& args = cmd->getArgValues();
  copy (args.begin(), args.end(), argv.begin());

  // Return values
  Value retval = Unknown;

  if (name == "owprint") owprint (cmd->getArgValues());

  if (name == "RA_DIG") unimplemented("RA_DIG");

  else {
    cerr << "Invalid command " << name << endl;
  }

  m_execInterface.handleCommandAck(cmd, COMMAND_SENT_TO_SYSTEM);
  if (retval != Unknown) m_execInterface.handleCommandReturn(cmd, retval);
  m_execInterface.notifyOfExternalEvent();
  debugMsg("OwAdapter:executeCommand", " " << name << " complete");
}


///////////////////////////// State support //////////////////////////////////

Value OwAdapter::fetch (const string& state_name, const vector<Value>& args)
{
  debugMsg("OwAdapter:fetch",
           " called on " << state_name << " with " << args.size() << " args");
  Value retval;
  string acid; // reused

  // NOTE: A more streamlined approach to dispatching on state name
  // would be nice.

  // Flight State

  // TODO: condense the following 3 lookups into one.
  if (state_name == "TrenchLength") {
    stubbed_lookup ("TrenchLength", "10");
    retval = 10;
  }
  else if (state_name == "TrenchIdentified") {
    stubbed_lookup ("TrenchIdentified", "true");
    retval = true;
  }
  else {
    cerr << "invalid state: " << state_name << endl;
    retval = Unknown;
  }

  debugMsg("OwAdapter:fetch", " " << state_name << " returning " << retval);
  return retval;
}


void OwAdapter::lookupNow (const State& state, StateCacheEntry& entry)
{
  debugMsg("OwAdapter:lookupNow", "Called with state " << state.name());
  entry.update(fetch(state.name(), state.parameters()));
}


void OwAdapter::subscribe(const State& state)
{
  debugMsg("OwAdapter:subscribe", " to state " << state.name());
  m_subscribedStates.insert(state);
}


void OwAdapter::unsubscribe (const State& state)
{
  debugMsg("OwAdapter:unsubscribe", " from state " << state.name());
  m_subscribedStates.erase(state);
}

extern "C" {
  void initOwAdapter() {
    REGISTER_ADAPTER(OwAdapter, "OwAdapter");
  }
}
