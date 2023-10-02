// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Ow_Plexil_Subscriber
#define Ow_Plexil_Subscriber

// This is a barebones publish-subscribe facility for PLEXIL.  It
// provides a set of subscription functions specific to various
// combinations of a PLEXIL Lookup's return type and parameters.  Only
// those used by OceanWATERS are defined.

// To do: incorporate the new template-based version of this pattern developed
// in the summer of 2020 by Albert Kutsyy.

#include <string>
#include <vector>

// Subscriber types
typedef void (* SubscribeBool) (const std::string& state_name, bool val);
typedef void (* SubscribeInt) (const std::string& state_name, int val);
typedef void (* SubscribeDouble) (const std::string& state_name, double val);
typedef void (* SubscribeString) (const std::string& state_name,
                                  const std::string& val);
typedef void (* SubscribeDoubleVector) (const std::string& state_name,
                                        const std::vector<double>& vals);
typedef void (* SubscribeBoolFromString) (const std::string& state_name,
                                          bool val, const std::string& arg);
typedef void (* SubscribeDoubleFromInt) (const std::string& state_name,
                                         double val, int arg);

// Setters for subscribers of each supported type signature
void setSubscriber (SubscribeBool);
void setSubscriber (SubscribeInt);
void setSubscriber (SubscribeDouble);
void setSubscriber (SubscribeString);
void setSubscriber (SubscribeDoubleVector);
void setSubscriber (SubscribeBoolFromString);
void setSubscriber (SubscribeDoubleFromInt);

// Publish a state name, which notifies the subscriber.
void publish (const std::string& state_name, bool val);
void publish (const std::string& state_name, int val);
void publish (const std::string& state_name, double val);
void publish (const std::string& state_name, const std::string& val);
void publish (const std::string& state_name, const std::vector<double>& vals);
void publish (const std::string& state_name, bool val, const std::string& arg);
void publish (const std::string& state_name, double val, int arg);

#endif
