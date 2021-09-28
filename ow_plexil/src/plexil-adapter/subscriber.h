// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Plexil_Subscriber
#define Ow_Plexil_Subscriber

// This is a barebones publish-subscribe facility for PLEXIL.  It provides a set
// of subscription functions specific to various combinations of return type and
// parameters.

// To do: incorporate the new template-based version of this pattern developed
// in the summer of 2020 by Albert Kutsyy.

#include <string>
#include <vector>
using std::string;
using std::vector;

// Subscriber types
typedef void (* SubscribeBool) (const string& state_name, bool val);
typedef void (* SubscribeDouble) (const string& state_name, double val);
typedef void (* SubscribeString) (const string& state_name,
                                  const string& val);
typedef void (* SubscribeBoolString) (const string& state_name,
                                      bool val, const string& arg);
typedef void (* SubscribeDoubleVector) (const string& state_name,
                                      vector<double> vals);


// Setters for subscribers of each supported type signature
void setSubscriber (SubscribeBool);
void setSubscriber (SubscribeDouble);
void setSubscriber (SubscribeString);
void setSubscriber (SubscribeBoolString);
void setSubscriber (SubscribeDoubleVector);

// Publish a state name, which notifies the subscriber.
void publish (const string& state_name, bool val);
void publish (const string& state_name, double val);
void publish (const string& state_name, const string& val);
void publish (const string& state_name, bool val, const string& arg);
void publish (const string& state_name, vector<double> vals);
using std::vector;

#endif
