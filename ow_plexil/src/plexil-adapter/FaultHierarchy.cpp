#include "FaultHierarchy.h"

#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <vector>
#include <iostream>
#include <memory>
//#include "/home/keegan/plexil/src/third-party/pugixml/src/pugixml.hpp"
#include "pugixml.hpp"


FaultHierarchy::FaultHierarchy()
{
/*  std::vector<std::string> test_names = {"fault1", "fault2", "fault3", "fault4", "fault5"};

  std::vector<std::string> sub1  = {"fault2", "fault3", "fault4"};
  std::vector<std::string> sub2  = {"fault1",  "fault4"};
  std::vector<std::string> sub3  = {"fault4"};
  std::vector<std::string> sub4  = {"fault3"};

  for (int i = 0; i < test_names.size(); i++){
    m_fault_model[test_names[i]] = fault();
    m_fault_model[test_names[i]].name = test_names[i];
  }
  m_fault_model["fault1"].subfaults = sub1;
  m_fault_model["fault2"].subfaults = sub2;
  m_fault_model["fault3"].subfaults = sub3;
  m_fault_model["fault4"].subfaults = sub4;
  //DebugPrint();
  //
  */
  const char* source = "xmltest.xml";
  parseXML(source);
  DebugPrint();

}

void FaultHierarchy::parseXML(const char* file_name){
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(file_name);

  if (result)
  {
      std::cout << "XML [" << file_name << "] parsed without errors, attr value: [" << doc.child("node").attribute("attr").value() << "]\n\n";
  }
  else
  {
      std::cout << "XML [" << file_name << "] parsed with errors, attr value: [" << doc.child("node").attribute("attr").value() << "]\n";
      std::cout << "Error description: " << result.description() << "\n";
      std::cout << "Error offset: " << result.offset << " (error at [..." << (file_name + result.offset) << "]\n\n";
  }

  pugi::xml_node subsystems = doc.child("System").child("Subsystems");
  for (pugi::xml_node subsystem = subsystems.child("Subsystem"); subsystem; subsystem = subsystem.next_sibling("Subsystem"))
  {
    std::string subsystem_name = subsystem.attribute("Name").value();
    std::string subsystem_severity_low = subsystem.child("SeverityThreshold").attribute("Low").value();
    std::string subsystem_severity_med = subsystem.child("SeverityThreshold").attribute("Medium").value();
    std::string subsystem_severity_high = subsystem.child("SeverityThreshold").attribute("High").value();

    if (subsystem_name.empty() ||  subsystem_severity_low.empty() || subsystem_severity_med.empty()
        || subsystem_severity_high.empty() ){
      std::cout << "WARNING: FAILED TO READ A SUBSYSTEM " << subsystem_name << "\n";
      continue;
    }

    if (m_subsystems.find(subsystem_name) == m_subsystems.end()){
      m_subsystems[subsystem_name] = Subsystem();
      m_subsystems[subsystem_name].name = subsystem_name;
      m_subsystems[subsystem_name].severity_threshold["Low"] = std::stoi(subsystem_severity_low);
      m_subsystems[subsystem_name].severity_threshold["Medium"] = std::stoi(subsystem_severity_med);
      m_subsystems[subsystem_name].severity_threshold["High"] = std::stoi(subsystem_severity_high);
    }
    else{
      continue;
    }
 
    std::cout << "Subsystem " << subsystem.attribute("Name").value() << "\n";
    pugi::xml_node sys_affected_subsystems = subsystem.child("AffectedSubsystems");

    for (pugi::xml_node affected_subsystem = sys_affected_subsystems.child("Sys"); affected_subsystem;
         affected_subsystem = affected_subsystem.next_sibling("Sys")){
      if (affected_subsystem.empty()){
        std::cout << "WARNING: FAILED TO READ AN AFFECTED SUBSYSTEM " << affected_subsystem << "\n";
        continue;
      }
      std::string affected_subsystem_name = affected_subsystem.attribute("Name").value();
      m_subsystems[subsystem_name].affected_subsystems.push_back(affected_subsystem_name);
      std::cout << "SYS AffectedSubsystems " << affected_subsystem.attribute("Name").value() << "\n";
    }
    for (pugi::xml_node fault_group = subsystem.child("FaultGroup"); fault_group; 
         fault_group = fault_group.next_sibling("FaultGroup")){
      std::string fault_group_name = fault_group.attribute("Name").value();
      std::string fault_group_severity = fault_group.attribute("Severity").value();
      std::string fault_group_severity_low = fault_group.child("SeverityThreshold").attribute("Low").value();
      std::string fault_group_severity_med = fault_group.child("SeverityThreshold").attribute("Medium").value();
      std::string fault_group_severity_high = fault_group.child("SeverityThreshold").attribute("High").value();

      if (fault_group_name.empty() || fault_group_severity.empty() || fault_group_severity_low.empty() 
          ||fault_group_severity_med.empty() ||fault_group_severity_high.empty() ){
        std::cout << "WARNING: FAILED TO READ A FAULTGROUP " << fault_group_name << "\n";
        continue;
      }

      if (m_fault_groups.find(fault_group_name) == m_fault_groups.end()){
        m_fault_groups[fault_group_name] = FaultGroup();
        m_fault_groups[fault_group_name].name = fault_group_name;
        m_fault_groups[fault_group_name].fault_group_severity = fault_group_severity;
        m_fault_groups[fault_group_name].severity_threshold["Low"] = std::stoi(fault_group_severity_low);
        m_fault_groups[fault_group_name].severity_threshold["Medium"] = std::stoi(fault_group_severity_med);
        m_fault_groups[fault_group_name].severity_threshold["High"] = std::stoi(fault_group_severity_high);
        m_subsystems[subsystem_name].fault_groups.push_back(fault_group_name);
      }
      else{
        continue;
      }
 
      std::cout << "FaultGroup " << fault_group.attribute("Name").value() << "\n";
      std::cout << "FaultGroup Severity " << fault_group_severity << "\n";

      pugi::xml_node faults = fault_group.child("Faults");
      pugi::xml_node affected_subsystems = fault_group.child("AffectedSubsystems");
      for (pugi::xml_node fault = faults.child("Fault"); fault; fault = fault.next_sibling("Fault")){
        std::string fault_name = fault.attribute("Name").value();
        std::string fault_severity = fault.attribute("Severity").value();
        if (fault_name.empty() || fault_severity.empty()){
          std::cout << "WARNING: FAILED TO READ A FAULT IN FAULt GROUP " << fault_group_name << "\n";
          continue;
        }

        if (m_faults.find(fault_name) == m_faults.end()){
          m_faults[fault_name] = Fault();
          m_faults[fault_name].name = fault_name;
        }
        m_fault_groups[fault_group_name].faults.push_back(fault_name);
        m_faults[fault_name].affected_fault_groups.push_back(std::make_pair(fault_group_name, fault_severity));
        std::cout << "Fault " << fault_name << "\n";
        std::cout << "Fault Severity" << fault_severity << "\n";

      }
      for (pugi::xml_node affected_subsystem = affected_subsystems.child("Sys"); affected_subsystem;
           affected_subsystem = affected_subsystem.next_sibling("Sys")){
        if (affected_subsystem.empty()){
          std::cout << "WARNING: FAILED TO READ AN AFFECTED SUBSYSTEM " << affected_subsystem << "\n";
          continue;
        }
        std::string affected_subsystem_name = affected_subsystem.attribute("Name").value();
        m_fault_groups[fault_group_name].affected_subsystems.push_back(affected_subsystem_name);
        std::cout << "AffectedSubsystems " << affected_subsystem.attribute("Name").value() << "\n";
      }
 
    }
  }


}


void FaultHierarchy::DebugPrint(){
  std::cout << "FAULTS: " << std::endl;
  for (auto fault: m_faults){
    std::cout << "Name:" << fault.second.name << " Status:" << fault.second.status << " FaultGroups: ";
    for (auto i: m_faults[fault.second.name].affected_fault_groups){
      std::cout << " (" << i.first << ", " << i.second << ")     ";
    }
    std::cout << std::endl;
    std::cout << std::endl;
  }
  std::cout << "FAULT GROUPS: " << std::endl;
  for (auto fault_group: m_fault_groups){
    std::cout << "Name:" << fault_group.second.name << " Status:" << fault_group.second.status << 
      " HierarchyFault:" << fault_group.second.hierarchy_faulted << " LocalFault:" << fault_group.second.locally_faulted << 
      " FaultGroupSeverity:" << fault_group.second.fault_group_severity << " FaultGroups: ";
    std::cout << std::endl;
    std::cout << "CURRENT FAULTGROUP SEVERITY: " << " LOW:" << fault_group.second.current_severity["Low"]
              << " MED:" << fault_group.second.current_severity["Medium"] 
              << " HIGH:" << fault_group.second.current_severity["High"] << std::endl; 
    std::cout << "FAULTGROUP SEVERITY THRESHOLD: " << " LOW:" << fault_group.second.severity_threshold["Low"]
              << " MED:" << fault_group.second.severity_threshold["Medium"] 
              << " HIGH:" << fault_group.second.severity_threshold["High"] << std::endl; 

    std::cout << "AffectedSubsystems: ";
    for (auto i: m_fault_groups[fault_group.second.name].affected_subsystems){
      std::cout << " " << i << " " << "     ";
    }
    std::cout << std::endl;
    std::cout << std::endl;
 
  }

  std::cout << "SUBSYSTEMS: " << std::endl;
  for (auto subsystem: m_subsystems){
    std::cout << "Name:" << subsystem.second.name << " Status:" << subsystem.second.status << 
      " HierarchyFault:" << subsystem.second.hierarchy_faulted << " LocalFault:" << subsystem.second.locally_faulted << " FaultGroups: ";
    std::cout << std::endl;
    std::cout << "CURRENT SUBSYSTEM SEVERITY: " << " LOW:" << subsystem.second.current_severity["Low"]
              << " MED:" << subsystem.second.current_severity["Medium"] 
              << " HIGH:" << subsystem.second.current_severity["High"] << std::endl; 
    std::cout << "SUBSYSTEM SEVERITY THRESHOLD: " << " LOW:" << subsystem.second.severity_threshold["Low"]
              << " MED:" << subsystem.second.severity_threshold["Medium"] 
              << " HIGH:" << subsystem.second.severity_threshold["High"] << std::endl; 

    std::cout << "AffectedSubsystems: ";
    for (auto i: m_subsystems[subsystem.second.name].affected_subsystems){
      std::cout << " " << i << " " << "     ";
    }
    std::cout << std::endl;
    std::cout << std::endl;
 
  }


}

void FaultHierarchy::updateFaultStatus(std::string name, int status){
  if (m_faults.find(name) == m_faults.end()){
    std::cout << "WARNING: FAULT NAME: " << name << " DOES NOT EXIST" << std::endl;
  }
  if (m_faults[name].status == status){
    return;
  }
  m_faults[name].status = status;

  for (auto related_fg: m_faults[name].affected_fault_groups){
    updateFaultGroupStatus(related_fg.first, status, related_fg.second);
  }


}

void FaultHierarchy::updateFaultGroupStatus(std::string name, int status, std::string severity){
  int prev_fg_status = m_fault_groups[name].status;
  if (status == 1){
    m_fault_groups[name].current_severity[severity] += 1;
    if (m_fault_groups[name].current_severity[severity] >= 
        m_fault_groups[name].severity_threshold[severity]){
      m_fault_groups[name].locally_faulted = 1;
      m_fault_groups[name].status = 1;
    }
  }
  else{
    m_fault_groups[name].current_severity[severity] -= 1;
    if (m_fault_groups[name].current_severity["Low"] < m_fault_groups[name].severity_threshold["Low"]
        && m_fault_groups[name].current_severity["Medium"] < m_fault_groups[name].severity_threshold["Medium"]
        && m_fault_groups[name].current_severity["High"] < m_fault_groups[name].severity_threshold["High"]){
      m_fault_groups[name].locally_faulted = 0;
      if (m_fault_groups[name].hierarchy_faulted == 0){
        m_fault_groups[name].status = 0;
      }
    }
  }
  if (prev_fg_status != m_fault_groups[name].status){
    for (auto subsystem_name: m_fault_groups[name].affected_subsystems){
      cascadeSubsystemFaults(subsystem_name, status);
      return true;
    }
  }
  return false;
}


void FaultHierarchy::cascadeSubsystemFaults(std::string subsystem_name, int status){
  std::unordered_set<std::string> visited;
  std::stack<std::string> nodes;

  visited.insert(subsystem_name);
  nodes.push(subsystem_name);
  
    // BFS search through all fault dependencies
    while(!nodes.empty()){
      std::string current_node = nodes.top();
      nodes.pop();

      for (auto i: m_subsystems[current_node].affected_subsystems){
        if (visited.insert(i).second){
          // All subfaults need to increment or decrement their hierarchical_fault flag
          if (status){
            m_subsystems[i].hierarchy_faulted += 1;
          }
          else{
            m_subsystems[i].hierarchy_faulted -= 1;
          }
          // Update subfault status flag based on updated hierarchical_fault flag
          if (m_subsystems[i].hierarchy_faulted > 0 || m_subsystems[i].hierarchy_faulted == 1){
            m_subsystems[i].status = 1;
          }
          else{
            m_subsystems[i].status = 0;
          }
          nodes.push(i);
        }
      }
    }
}

void FaultHierarchy::updateFaultModel(const std::string name, const bool status, const int severity){
/*  std::unordered_set<std::string> visited;
  std::stack<std::string> nodes;

  if (m_fault_model.find(name) == m_fault_model.end()){
    std::cout << "ERROR: COULD NOT FIND FAULT NAME: " << name << std::endl;
    return;
  }
  if(m_fault_model[name].local_fault == status){
    std::cout << "NO UPDATE NEEDED FOR: " << name << std::endl;
    return;
  }

  //initialize visited set and nodes stack for BFS
  visited.insert(name);
  nodes.push(name);
  m_fault_model[name].local_fault = status;
  
  if (severity >= m_fault_model[name].severity_threshold){
    // BFS search through all fault dependencies
    while(!nodes.empty()){
      std::string current_node = nodes.top();
      nodes.pop();

      for (auto i: m_fault_model[current_node].subfaults){
        if (visited.insert(i).second){
          // All subfaults need to increment or decrement their hierarchical_fault flag
          if (status){
            m_fault_model[i].hierarchical_faults += 1;
          }
          else{
            m_fault_model[i].hierarchical_faults -= 1;
          }
          // Update subfault status flag based on updated hierarchical_fault flag
          if (m_fault_model[i].hierarchical_faults > 0 || m_fault_model[i].local_fault == 1){
            m_fault_model[i].status = 1;
          }
          else{
            m_fault_model[i].status = 0;
          }
          nodes.push(i);
        }
      }
      }
  }
  */
}


/*FaultHierarchy::FaultHierarchy()
{
  std::unordered_map<std::string, std::unique_ptr<fault>> fault_model;
  std::vector<std::string> test_names = {"fault1", "fault2", "fault3", "fault4"};

  for (int i = 0; i < test_names.size(); i++){
    fault_model[test_names[i]] = std::make_unique<fault>();
    fault_model[test_names[i]]->name = test_names[i];
    fault_model[test_names[i]]->subfaults.push_back("TEST1");
    fault_model[test_names[i]]->subfaults.push_back("TEST2");
    fault_model[test_names[i]]->subfaults.push_back("TEST3");
  }

  for (auto i: test_names){
    std::cout << fault_model[i]->name << std::endl;
    for (auto j: fault_model[i]->subfaults){
      std::cout << j << " ";
    }
  }

}
*/
