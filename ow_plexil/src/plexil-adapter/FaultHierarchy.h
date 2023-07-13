#ifndef Fault_Hierarchy
#define Fault_Hierarchy

#include <unordered_map>
#include <vector>

// PLEXIL Fault Hierarchy Class for use in the Interface class with fault lookups.

struct fault {
  std::string name;
  int severity_threshold = 0;
  bool local_fault = 0;
  int hierarchical_faults = 0;
  bool status = 0;
  std::vector<std::string> subfaults;
};

class FaultHierarchy;

class FaultHierarchy
{
public:
  // No default constructor, only this specialized one.
  FaultHierarchy ();
  ~FaultHierarchy() = default;
  FaultHierarchy (const FaultHierarchy&) = delete;
  FaultHierarchy& operator= (const FaultHierarchy&) = delete;
  void updateFaultModel(const std::string name, const bool status, const int severity);
  void DebugPrint();


private:
  std::unordered_map<std::string, fault> m_fault_model;

};

#endif
