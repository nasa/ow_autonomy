#ifndef Fault_Dependencies
#define Fault_Dependencies

#include <unordered_map>
#include <utility>
#include <vector>

// Fault Dependencies Class for use in the Interface class with fault lookups.

struct Fault {
  std::string name;
  std::string parent_subsystem;
  int faulty = 0;
  std::vector<std::pair<std::string, std::string>> impacts;
};

struct Procedure {
  std::string name;
  int inoperable = 0;
};

struct Subsystem {
  std::string name;
  int local_fault = 0;
  int non_local_fault = 0;
  int faulty = 0;
  int inoperable = 0;
  std::vector<std::string> faults;
  std::vector<std::pair<std::string, std::string>> impacts;
};

class FaultDependencies
{
public:

  FaultDependencies (std::string file_name, bool verbose_flag);
  ~FaultDependencies() = default;
  FaultDependencies (const FaultDependencies&) = delete;
  FaultDependencies& operator= (const FaultDependencies&) = delete;

  // get and update functions
  void updateFault(std::string name, int status);
  std::vector<std::string> getActiveFaults(std::string name);
  bool checkIsOperable(std::string);
  bool checkIsFaulty(std::string);
  void DebugPrint();

private:

  // internal functions
  void updateSubsystem(std::string name, int status, std::string parent);
  void cascadeFault(std::string name, int status);
  void parseXML(const char* file_name);

  // verbose debug print flag
  bool m_verbose_flag = true;

  // primary fault hierarchy data structures
  std::unordered_map<std::string, Fault> m_faults;
  std::unordered_map<std::string, Procedure> m_procedures;
  std::unordered_map<std::string, Subsystem> m_subsystems;

};

#endif
