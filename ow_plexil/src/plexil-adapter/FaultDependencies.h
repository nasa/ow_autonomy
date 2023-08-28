#ifndef Fault_Dependencies
#define Fault_Dependencies

#include <unordered_map>
#include <utility>
#include <vector>

// Fault Dependencies Class for use in the Interface class with fault lookups.

struct Fault {
  std::string name;
  std::string parent_subsystem;
  bool faulty = 0;
  std::vector<std::pair<std::string, std::string>> impacts;
};

struct Procedure {
  std::string name;
  int num_inoperable = 0;
};

struct Subsystem {
  std::string name;
  int num_local_faults = 0; // Number of active faults in local subsystem
  int num_non_local_faults = 0; // Number of active faults that impact this subsystem which are non-local
  int num_inoperable = 0; // Number of active faults that impact this subsystem (Local or Non-local).
  bool faulty = false; // Flag for if the subsystem has any local faults
  std::vector<std::string> faults;
  std::vector<std::pair<std::string, std::string>> impacts;
};

class FaultDependencies
{
public:

  FaultDependencies (const std::string &file_name, bool verbose_flag);
  FaultDependencies () = delete;
  ~FaultDependencies() = default;
  FaultDependencies (const FaultDependencies&) = delete;
  FaultDependencies& operator= (const FaultDependencies&) = delete;

  // get and update functions
  void updateFault(const std::string &name, int status);
  std::vector<std::string> getActiveFaults(const std::string &name) const;
  bool checkIsOperable(const std::string &name) const;
  bool checkIsFaulty(const std::string &name) const;
  void DebugPrint() const;

private:

  // internal functions
  void updateSubsystem(const std::string &name, int status, const std::string &parent);
  void cascadeFault(const std::string &name, int status);
  bool parseXML(const std::string &file);

  // verbose debug print flag
  bool m_verbose_flag = true;

  // primary fault hierarchy data structures
  std::unordered_map<std::string, Fault> m_faults;
  std::unordered_map<std::string, Procedure> m_procedures;
  std::unordered_map<std::string, Subsystem> m_subsystems;

};

#endif
