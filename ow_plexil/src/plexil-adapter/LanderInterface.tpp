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
                                         const std::string& general_name)
{
  // general_name is a PLEXIL lookup name that generalizes the
  // fault. This fault is the logical union of specific faults in the
  // fault map, i.e. it is true iff at least one specific fault is
  // active.

  bool general_fault = false;

  for (auto const& entry : fmap) {
    std::string specific_name = entry.first;
    T1 value = entry.second.first;
    bool fault_active = entry.second.second;
    bool faulty = (msg_val & value) == value;
    if (!fault_active && faulty) {
      ROS_WARN ("Fault in %s: %s", component.c_str(), specific_name.c_str());
      fmap[specific_name].second = true;
      general_fault = true;  // since there's at least one specific fault
      publish (specific_name, true);
      if (m_fault_dependencies) {
        m_fault_dependencies->updateFault(specific_name.c_str(), 1);
      }
    }
    else if (fault_active && !faulty) {
      ROS_WARN ("Resolved fault in %s: %s",
                component.c_str(),
                specific_name.c_str());
      fmap[specific_name].second = false;
      publish (specific_name, false);
      if (m_fault_dependencies) {
        m_fault_dependencies->updateFault(specific_name.c_str(), 0);
      }
    }
  }
  if (m_fault_dependencies) {
    m_fault_dependencies->updateFault(general_name.c_str(), general_fault);
  }
  publish (general_name, general_fault);
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
