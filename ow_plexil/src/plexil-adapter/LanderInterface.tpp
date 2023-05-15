// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// Implements template functions in LanderInterface.h and is only for
// inclusion by that file.

#ifndef LanderInterface_T
#define LanderInterface_T

#include "subscriber.h"

template <typename T1, typename T2>
void LanderInterface::updateFaultStatus (T1 msg_val, T2& fmap,
                                         const std::string& component,
                                         const std::string& state_name)
{
  for (auto const& entry : fmap) {
    std::string key = entry.first;
    T1 value = entry.second.first;
    bool fault_active = entry.second.second;
    bool faulty = (msg_val & value) == value;
    if (!fault_active && faulty) {
      ROS_WARN ("Fault in %s: %s", component.c_str(), key.c_str());
      fmap[key].second = true;
      publish (state_name, true);
    }
    else if (fault_active && !faulty) {
      ROS_WARN ("Resolved fault in %s: %s", component.c_str(), key.c_str());
      fmap[key].second = false;
      publish (state_name, false);
    }
  }
}

template <typename T>
bool LanderInterface::faultActive (const T& fmap) const
{
  for (auto const& entry : fmap) {
    if (entry.second.second) return true;
  }
  return false;
}

#endif
