// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Plexil_Subscriber
#define Ow_Plexil_Subscriber

// This is a barebones publish-subscribe facility for PLEXIL.  It provides a set
// of subscription functions specific to various combinations of return type and
// parameters.

#include <string>
using std::string;

// Subscriber types
typedef void (* SubscribeBool) (const string& state_name, bool val);
typedef void (* SubscribeDouble) (const string& state_name, double val);
typedef void (* SubscribeString) (const string& state_name,
                                  const string& val);
typedef void (* SubscribeBoolString) (const string& state_name,
                                      bool val, const string& arg);

// Setters for subscribers of each supported type signature
void setSubscriber (SubscribeBool);
void setSubscriber (SubscribeDouble);
void setSubscriber (SubscribeString);
void setSubscriber (SubscribeBoolString);

// Publish a state name, which notifies the subscriber.
void publish (const string& state_name, bool val);
void publish (const string& state_name, double val);
void publish (const string& state_name, const string& val);
void publish (const string& state_name, bool val, const string& arg);

#endif
