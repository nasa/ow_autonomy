// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#include "subscriber.h"

using std::string;
using std::vector;


// The subscribers.
// The naming convention for Lookups without parameters is:
//   Subscribe<value-type><param-type>...
// The naming convention for Lookups with parameters is:
//   Subscribe<value-type>From<param-type>...
// where value-type and param-type are ad hoc, CamelCase names.

static SubscribeBool SubscriberBool = nullptr;
static SubscribeInt SubscriberInt = nullptr;
static SubscribeDouble SubscriberDouble = nullptr;
static SubscribeString SubscriberString = nullptr;
static SubscribeDoubleVector SubscriberDoubleVector = nullptr;
static SubscribeBoolFromString SubscriberBoolFromString = nullptr;
static SubscribeIntFromString SubscriberIntFromString = nullptr;
static SubscribeDoubleFromInt SubscriberDoubleFromInt = nullptr;

void setSubscriber (SubscribeBool s) { SubscriberBool = s; }
void setSubscriber (SubscribeInt s) { SubscriberInt = s; }
void setSubscriber (SubscribeDouble s) { SubscriberDouble = s; }
void setSubscriber (SubscribeString s) { SubscriberString = s; }
void setSubscriber (SubscribeBoolFromString s) { SubscriberBoolFromString = s; }
void setSubscriber (SubscribeIntFromString s) { SubscriberIntFromString = s; }
void setSubscriber (SubscribeDoubleVector s) { SubscriberDoubleVector = s; }
void setSubscriber (SubscribeDoubleFromInt s) { SubscriberDoubleFromInt = s; }

// The overloaded publish function, one for each value/parameter
// combination found in this application.  NOTE: these assume the
// PLEXIL interface adapter has been initialized, otherwise the called
// subscriber functions will be null.

void publish (const string& state_name, bool val)
{
  SubscriberBool (state_name, val);
}

void publish (const string& state_name, int val)
{
  SubscriberInt (state_name, val);
}

void publish (const string& state_name, double val)
{
  SubscriberDouble (state_name, val);
}

void publish (const string& state_name, const string& val)
{
  SubscriberString (state_name, val);
}

void publish (const string& state_name, const vector<double>& vals)
{
  SubscriberDoubleVector (state_name, vals);
}

void publish (const string& state_name, bool val, const string& arg)
{
  SubscriberBoolFromString (state_name, val, arg);
}

void publish (const string& state_name, int val, const string& arg)
{
  SubscriberIntFromString (state_name, val, arg);
}

void publish (const string& state_name, double val, int arg)
{
  SubscriberDoubleFromInt (state_name, val, arg);
}
