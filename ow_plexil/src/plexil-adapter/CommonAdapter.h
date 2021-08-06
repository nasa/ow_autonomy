// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Common_Adapter
#define Common_Adapter

// PLEXIL Interface adapter base class to be specialized for each testbed, along
// with support utilities.

// PLEXIL API
#include "InterfaceAdapter.hh"
#include "Command.hh"
#include "Value.hh"

#include <set>
#include <map>

using namespace PLEXIL;

class CommonAdapter : public InterfaceAdapter
{
public:
  // No default constructor, only this specialized one.
  CommonAdapter (AdapterExecInterface&, const pugi::xml_node&);
  ~CommonAdapter ();
  CommonAdapter (const CommonAdapter&) = delete;
  CommonAdapter& operator= (const CommonAdapter&) = delete;

  virtual bool initialize();
  virtual bool start();
  virtual bool stop();
  virtual bool reset();
  virtual bool shutdown();
  virtual void invokeAbort(Command *cmd);
  virtual void subscribe(const State& state);
  virtual void unsubscribe(const State& state);
  void propagateValueChange (const State&, const std::vector<Value>&) const;

protected:
  bool isStateSubscribed(const State& state) const;
  std::set<State> m_subscribedStates;
};


///////////////////////////// Conveniences //////////////////////////////////

// A prettier name for the "unknown" value.
const Value Unknown;


//////////////////////////// Command Handling //////////////////////////////


// Unique ID for every instance of a command from a Plexil plan.
extern int CommandId;

// Record that combines a Plexil command instance with its executive interface
// and a flag indicating whether the command has been acknowledged.
using CommandRecord =
  std::tuple<Command*, AdapterExecInterface*, bool /* ack_sent */>;

enum CommandRecordFields {CR_COMMAND, CR_ADAPTER, CR_ACK_SENT};

std::unique_ptr<CommandRecord>&
new_command_record(Command*, AdapterExecInterface*);

// Registry of all commands currently in execution by testbed.
extern std::map<int, std::unique_ptr<CommandRecord>> CommandRegistry;

// Acknowledge a command issued by a Plexil plan, in a way that guarantees only
// one acknowledgment (acks are not idempotent).
void send_ack_once(CommandRecord&, bool skip=false);

// Function to call when a command finishes execution in testbed.
void command_status_callback (int id, bool success);

#endif
