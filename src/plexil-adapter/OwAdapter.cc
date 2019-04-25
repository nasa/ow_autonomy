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
using std::string;
using std::vector;
using std::list;
using std::copy;
using std::cerr;
using std::cout;
using std::endl;

// Domain
#include "OwSimInterface.hh"


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

///////////////////////////// Domain Support //////////////////////////////////

// We keep the domain interface as lightweight as possible!

static OwSimInterface* TheSimInterface = 0;


///////////////////////////// Member functions //////////////////////////////////


OwAdapter::OwAdapter(AdapterExecInterface& execInterface,
                     const pugi::xml_node& configXml) :
  InterfaceAdapter(execInterface, configXml)
{
  TheSimInterface = new OwSimInterface();
  debugMsg("OwAdapter", " created.");
}

OwAdapter::~OwAdapter ()
{
  if (TheSimInterface) delete TheSimInterface;
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

// NOTE: This macro, and the stub it implements, are temporary.
#define COMMAND_STUB(command)                   \
  if (name == #command) {                       \
    unimplemented(#command);                    \
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
  if (name == "DigTrench") {
    Real start_x, start_y, start_z, dump_x, dump_y, dump_z;
    Real depth, length, width, pitch, yaw;
      
    args[0].getValue(start_x);
    args[1].getValue(start_y);
    args[2].getValue(start_z);
    args[3].getValue(depth);
    args[4].getValue(length);
    args[5].getValue(width);
    args[6].getValue(pitch);
    args[7].getValue(yaw);
    args[8].getValue(dump_x);
    args[9].getValue(dump_y);
    args[10].getValue(dump_z);
    TheSimInterface->DigTrench (start_x, start_x, start_x, 
                            depth, length, width, pitch, yaw, 
                            dump_x, dump_y, dump_z);
  }
  else COMMAND_STUB(RA_COLLECT)
  else COMMAND_STUB(ALIGN_SAMPLE_AND_CAMERA)
  else {
    cerr << "Invalid command " << name << endl;
  }

  m_execInterface.handleCommandAck(cmd, COMMAND_SENT_TO_SYSTEM);
  m_execInterface.handleCommandAck(cmd, COMMAND_SUCCESS);
  if (retval != Unknown) m_execInterface.handleCommandReturn(cmd, retval);
  m_execInterface.notifyOfExternalEvent();
  debugMsg("OwAdapter:executeCommand", " " << name << " complete");
}


///////////////////////////// State support //////////////////////////////////

void OwAdapter::lookupNow (const State& state, StateCacheEntry& entry)
{
  debugMsg("OwAdapter:lookupNow", " called on " << state.name() << " with "
           << state.parameters().size() << " arguments");

  Value retval = Unknown;  // the value of the queried state

  if (! TheSimInterface->lookup(state.name(), state.parameters(), retval)) {
    cerr << "Invalid Lookup: " << state.name() << endl;
  }
  entry.update(retval);
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
  void initow_adapter() {
    REGISTER_ADAPTER(OwAdapter, "ow_adapter");
  }
}
