#ifndef Ow_Plexil_Subscriber
#define Ow_Plexil_Subscriber

// This is a barebones publish-subscribe facility.  It provides a set of
// subscription functions specific to various combinations of return type and
// parameters.

#include <string>

// Subscriber types
typedef void (* SubscribeBool) (const std::string& state_name, bool val);
typedef void (* SubscribeString) (const std::string& state_name,
                                  const std::string& val);
typedef void (* SubscribeBoolString) (const std::string& state_name,
                                      bool val, const std::string& arg);

// Setters for subscribers of each supported type signature
void setSubscriber (SubscribeBool);
void setSubscriber (SubscribeString);
void setSubscriber (SubscribeBoolString);

// Publish a state name, which notifies the subscriber.
void publish (const std::string& state_name, bool val);
void publish (const std::string& state_name, const std::string& val);
void publish (const std::string& state_name, bool val, const std::string& arg);

#endif
