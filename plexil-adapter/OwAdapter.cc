#include "OwAdapter.hh"

// PLEXIL API
#include <AdapterConfiguration.hh>
#include <AdapterFactory.hh>
#include <AdapterExecInterface.hh>
#include <ArrayImpl.hh>
#include <Debug.hh>
#include <Expression.hh>
#include <StateCacheEntry.hh>

// C++/C
#include <sstream>
#include <math.h>
using std::string;
using std::vector;
using std::list;
using std::copy;


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

  else {
    AosEvent(Event::Critical, "OwAdapter:executeCommand").out
      << "Invalid command " << name;
  }

  m_execInterface.handleCommandAck(cmd, COMMAND_SENT_TO_SYSTEM);
  if (retval != Unknown) m_execInterface.handleCommandReturn(cmd, retval);
  m_execInterface.notifyOfExternalEvent();
  debugMsg("OwAdapter:executeCommand", " " << name << " complete");
}


///////////////////////////// State support //////////////////////////////////

static int angle_between (float waypoint)
{
  // Stub
  return waypoint * 50;
}

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
    retval = FlightState::instance().lastElementAchieved ();
  }
  else if (state_name == "RouteFinished") {
    retval = ! FlightState::instance().clearance.pendingRoute ();
  }
  else if (state_name == "PendingRoute") {
    retval = FlightState::instance().clearance.pendingRoute ();
  }
  else if (state_name == "NewRouteSegment") {
    retval = FlightState::instance().clearance.newRouteSegment ();
  }
  else if (state_name == "CurrentRouteSegmentAchieved") {
    retval = FlightState::instance().clearance.currentRouteSegmentAchieved();
  }
  else if (state_name == "RadarVectored") {
    retval = FlightState::instance().clearance.radarVectored ();
  }
  else if (state_name == "RadarVectored2") {
    retval = FlightState::instance().radarVectored ();
  }
  else if (state_name == "ExpectingLateralClearance") {
    retval = FlightState::instance().clearance.expectingLateralClearance ();
  }
  else if (state_name == "ExpectingLateralClearance2") {
    retval = FlightState::instance().expectingLateralClearance ();
  }
  else if (state_name == "ExpectingAltitudeClearance") {
    retval = FlightState::instance().clearance.expectingAltitudeClearance ();
  }
  else if (state_name == "ClearanceAltitude") {
    retval = FlightState::instance().clearance.clearanceAltitude ();
  }
  else if (state_name == "ExpectedAltitude") {
    retval = FlightState::instance().clearance.expectedAltitude ();
  }
  else if (state_name == "MEA") {
    retval = FlightState::instance().clearance.mea();
  }
  else if (state_name == "MessageFromATC") {  // to be deprecated
    string message = theATC.getTextClearance ();
    retval = (message == "" ? Unknown : message);
  }
  else if (state_name == "TextMessageFromATC") retval = ""; // event only
  else if (state_name == "RouteFromDM") retval = "";  // event only
  else if (state_name == "VoiceMessageFromATC") {
    retval = theATC.voiceMessageArrived();
  }
  else if (state_name == "PreviousWaypoint") {
    string wp;
    args[0].getValue(wp);
    retval = FlightState::instance().clearance.previousWaypoint() == wp;
  }
  else if (state_name == "LastWaypointReached") {
    retval = FlightState::instance().prevWaypoint.name;
  }
  else if (state_name == "AtFinalApproachFix") {
    // STUB
    retval = false;
  }
  else if (state_name == "LostCommAdjustmentNeeded") {
    retval = false; // event only
  }

  // NEW

  else if (state_name == "ReachedCommandedHeading") {
    retval = FlightState::instance().reachedCommandedHeading();
  }
  else if (state_name == "ReachedCommandedAltitude") {
    retval = FlightState::instance().reachedCommandedAltitude();
  }
  else if (state_name == "ReachedCommandedWaypoint") {
    retval = FlightState::instance().reachedCommandedWaypoint();
  }
  else if (state_name == "ReachedAWaypoint") {
    retval = FlightState::instance().reachedAWaypoint();
  }
  // TODO: combine the following two
  else if (state_name == "AtClearanceLimit") {
    retval = FlightState::instance().atClearanceLimit();
  }
  else if (state_name == "ReachedClearanceLimit") {
    // Unlike the previous AtClearanceLimit, which is a state, this lookup is an
    // EVENT.
    retval = FlightState::instance().atClearanceLimit();
  }
  else if (state_name == "WithinRadioDistanceOfTower") {
    retval = true;  // STUB
  }
  else if (state_name == "ClearanceAmended") {
    //    retval = FlightState::instance().clearanceAmended();
    retval = false; // event only
  }
  else if (state_name == "ClearanceMissing") {
    retval = FlightState::instance().clearanceMissing();
  }
  else if (state_name == "OnlyAirportNext") {
    retval = FlightState::instance().onlyAirportNext();
  }
  else if (state_name == "Landed") {
    retval = FlightState::instance().phase == Landed;
  }
  else if (state_name == "Holding") {
    retval = FlightState::instance().holding();
  }
  else if (state_name == "EnteringTrafficPattern") {
    retval = false; // default value; true is only published
  }
  else if (state_name == "NextElementReady") {
    retval = false; // default value; true is only published
  }
  else if (state_name == "ReadyForNextElement") {
    retval = FlightState::instance().readyForNextRouteElement();
  }
  else if (state_name == "ReadyForNextLegElement") {
    retval = FlightState::instance().readyForNextLegElement();
  }
  
  // Terminal Procedure support

  else if (state_name == "CurrentAltitude") {
    retval = FlightInterface::instance()->currentAltitude();
  }
  else if (state_name == "CurrentHeading") {
    retval = FlightInterface::instance()->currentHeading();
  }
  else if (state_name == "ReachedWaypoint") {
    string wp;
    args[0].getValue(wp);
    retval = FlightInterface::instance()->reachedWaypoint(wp);
  }
  else if (state_name == "CurrentSpeed") {
    retval = FlightInterface::instance()->currentAirSpeed();
  }
  else if (state_name == "StallSpeed") {
    retval = 4;
  }
  else if (state_name == "TPAltitude") {
    retval = 25;
  }
  else if (state_name == "RunwayDesignation") {
    stubbed_lookup (state_name, "18");
    retval = 18;
  }
  else if (state_name == "AngleBetween") {
    if (args.size() == 1) {
      double waypoint;
      args[0].getValue(waypoint);
      std::cout << "AngleBetween looked up with " << waypoint << std::endl;
      retval = angle_between(waypoint);
    }
    else AosEvent(Event::Critical, "OwAdapter::fetch").out
           << "Expects only one argument: " << state_name;
  }

  // Support for Langley

  else if (state_name == "ScenarioReady") {
    // Stub.  Should return true for Ames scenarios.  Needs to be triggered by
    // CFS message for Langley scenarios.
    retval = true;
  }

  else {
    AosEvent(Event::Critical, "OwAdapter::fetch").out
      << "invalid state: " << state_name;
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
  // NOTE: This function IS called, even though cppcheck doesn't think so!
  void initOwAdapter() {
    REGISTER_ADAPTER(OwAdapter, "OwAdapter");
  }
}
