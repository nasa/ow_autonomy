#ifndef Ow_Plexil_Subscriber
#define Ow_Plexil_Subscriber

// This is a barebones publish-subscribe facility.  It provides a set of
// subscription functions specific to various combinations of return type and
// parameters.

// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

#include <string>
using std::string;

// Subscriber types
typedef void (* SubscribeBool) (const string& state_name, bool val);
typedef void (* SubscribeString) (const string& state_name,
                                  const string& val);
typedef void (* SubscribeBoolString) (const string& state_name,
                                      bool val, const string& arg);

// Setters for subscribers of each supported type signature
void setSubscriber (SubscribeBool);
void setSubscriber (SubscribeString);
void setSubscriber (SubscribeBoolString);

// Publish a state name, which notifies the subscriber.
void publish (const string& state_name, bool val);
void publish (const string& state_name, const string& val);
void publish (const string& state_name, bool val, const string& arg);

#endif
