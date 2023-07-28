#ifndef Fault_Hierarchy
#define Fault_Hierarchy

#include <unordered_map>
#include <utility>
#include <vector>

// PLEXIL Fault Hierarchy Class for use in the Interface class with fault lookups.

struct Fault {
  std::string name;
  int status = 0;
  std::vector<std::pair<std::string, std::string>> affected_subsystems;
};

struct Subsystem {
  std::string name;
  int hierarchy_faulted = 0;
  int locally_faulted = 0;
  int status = 0;
  std::vector<std::string> faults;
  std::unordered_map<std::string, int> severity_threshold = 
    {
      {"Low", 0},
      {"Medium", 0},
      {"High", 0}
    };
  std::unordered_map<std::string, int> current_severity =
    {
      {"Low", 0},
      {"Medium", 0},
      {"High", 0}
    };
  std::vector<std::pair<std::string, bool>> affected_subsystems;
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
  void updateFaultModel(std::string name, int status);
  void updateSubsystemStatus(std::string name,  int status, std::string severity);
  void cascadeSubsystemFaults(std::string name, int status);
  void DebugPrint();


private:
  void parseXML(const char* file_name);

  std::unordered_map<std::string, Fault> m_faults;
  std::unordered_map<std::string, Subsystem> m_subsystems;

};

#endif
