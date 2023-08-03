#ifndef Fault_Hierarchy
#define Fault_Hierarchy

#include <unordered_map>
#include <utility>
#include <vector>

// Fault Hierarchy for use in the Interface class with fault lookups.

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

class FaultHierarchy
{
public:

  FaultHierarchy (std::string file_name, bool verbose_flag);
  ~FaultHierarchy() = default;
  FaultHierarchy (const FaultHierarchy&) = delete;
  FaultHierarchy& operator= (const FaultHierarchy&) = delete;

  // get and update functions
  std::vector<bool> getSubsystemStatus(std::string name);
  std::vector<std::string> getActiveFaults(std::string name);
  void updateFaultHierarchy(std::string name, int status);

private:

  // internal functions
  void updateSubsystemStatus(std::string name,  int status, std::string severity);
  void cascadeSubsystemFaults(std::string name, int status);
  void DebugPrint();
  void parseXML(const char* file_name);

  // verbose debug print flag
  bool m_verbose_flag = true;

  // primary fault hierarchy data structures
  std::unordered_map<std::string, Fault> m_faults;
  std::unordered_map<std::string, Subsystem> m_subsystems;

};

#endif
