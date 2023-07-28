#include "FaultHierarchy.h"

#include <unordered_map>
#include <iomanip>      // std::setw
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
  const char* source = "xmltest2.xml";
  parseXML(source);
  DebugPrint();
  updateFaultModel("Overdraw",1);
  DebugPrint();
  updateFaultModel("NoCharge",1);
  DebugPrint();
  updateFaultModel("scoop_yaw_joint_locked_failure",1);
  DebugPrint();
  updateFaultModel("NoCharge",0);
  DebugPrint();
  updateFaultModel("Overdraw",0);
  DebugPrint();
  updateFaultModel("scoop_yaw_joint_locked_failure",0);
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
    pugi::xml_node faults = subsystem.child("Faults");

    for (pugi::xml_node affected_subsystem = sys_affected_subsystems.child("Sys"); affected_subsystem;
         affected_subsystem = affected_subsystem.next_sibling("Sys")){
      if (affected_subsystem.empty()){
        std::cout << "WARNING: FAILED TO READ AN AFFECTED SUBSYSTEM " << affected_subsystem << "\n";
        continue;
      }
      std::string affected_subsystem_name = affected_subsystem.attribute("Name").value();
      std::string affected_subsystem_cascade_flag_str = affected_subsystem.attribute("Cascade").value();
      bool cascade_flag = false;
      if (affected_subsystem_cascade_flag_str == "True"){
        cascade_flag = true;
      }
      else if (affected_subsystem_cascade_flag_str == "False"){
        cascade_flag = false;
      }
      else{
        std::cout << "WARNING: FAILED TO READ CASCADE FLAG ON AFFECTED SUBSYSTEM " << affected_subsystem << "\n";
        continue;
      }

      m_subsystems[subsystem_name].affected_subsystems.push_back(std::make_pair(affected_subsystem_name, cascade_flag));
      std::cout << "SYS AffectedSubsystems " << affected_subsystem.attribute("Name").value() << "\n";
    }

    for (pugi::xml_node fault = faults.child("Fault"); fault; fault = fault.next_sibling("Fault")){
      std::string fault_name = fault.attribute("Name").value();
      std::string fault_severity = fault.attribute("Severity").value();
      if (fault_name.empty() || fault_severity.empty()){
        std::cout << "WARNING: FAILED TO READ A FAULT IN SUBSYSTEM " << subsystem_name << "\n";
        continue;
      }

      if (m_faults.find(fault_name) == m_faults.end()){
        m_faults[fault_name] = Fault();
        m_faults[fault_name].name = fault_name;
      }
      m_subsystems[subsystem_name].faults.push_back(fault_name);
      m_faults[fault_name].affected_subsystems.push_back(std::make_pair(subsystem_name, fault_severity));
      std::cout << "Fault " << fault_name << "\n";
      std::cout << "Fault Severity" << fault_severity << "\n";

    }
  }
}


void FaultHierarchy::DebugPrint(){
  std::cout << "FAULTS: " << std::endl;
  std::cout << "------------------------------------------------------------------------"
            << "------------------------------------------------------------------------" 
            << std::endl;
  std::cout << std::setw(40) << std::left << "NAME" << std::setw(20) << "STATUS" << std::setw(27) << "SUBSYSTEMS/SEVERITY" 
  <<  std::endl;
  std::cout << "------------------------------------------------------------------------"
            << "------------------------------------------------------------------------" 
            << std::endl;
  for (auto fault: m_faults){
    std::cout << std::left << std::setw(40) << fault.second.name << std::setw(20) << fault.second.status;
    for (auto i: m_faults[fault.second.name].affected_subsystems){
      std::cout << std::setw(1) << "(" << i.first << ", " << i.second << ") ";
    }
    std::cout << std::endl;
  }

  std::cout << "\n" << std::endl;
  std::cout << "SUBSYSTEMS: " << std::endl;
  std::cout << "------------------------------------------------------------------------"
            << "------------------------------------------------------------------------" 
            << std::endl;

  std::cout << std::setw(20) << std::left << "NAME" << std::setw(20) << "STATUS" << std::setw(27) << "HIERARCHY FAULT" 
  << std::setw(20) << "LOCAL FAULT" << std::setw(10) << "LOW" << std::setw(10) << "MED"<< std::setw(10) << "HIGH"<<
  std::setw(10) << "AFFECTED SUBSYSTEMS" << std::endl; 
  std::cout << "------------------------------------------------------------------------"
            << "------------------------------------------------------------------------" 
            << std::endl;
  for (auto subsystem: m_subsystems){
    std::cout << std::left << std::setw(20) << subsystem.second.name << std::setw(20) << subsystem.second.status << std::setw(27) << 
     subsystem.second.hierarchy_faulted << std::setw(20) << subsystem.second.locally_faulted 
     << std::setw(1) << subsystem.second.current_severity["Low"] << "/" << std::setw(8) << subsystem.second.severity_threshold["Low"]
     << std::setw(1) << subsystem.second.current_severity["Medium"] << "/" << std::setw(8) << subsystem.second.severity_threshold["Medium"]
     << std::setw(1) << subsystem.second.current_severity["High"] << "/" << std::setw(8) << subsystem.second.severity_threshold["High"];

    for (auto i: m_subsystems[subsystem.second.name].affected_subsystems){
      std::cout << i.first << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

void FaultHierarchy::updateFaultModel(std::string name, int status){
  if (m_faults.find(name) == m_faults.end()){
    std::cout << "WARNING: FAULT NAME: " << name << " DOES NOT EXIST" << std::endl;
    return;
  }
  if (m_faults[name].status == status){
    return;
  }
  m_faults[name].status = status;

  for (auto affected_sub: m_faults[name].affected_subsystems){
    if (m_subsystems.find(affected_sub.first) != m_subsystems.end()){
      updateSubsystemStatus(affected_sub.first, status, affected_sub.second);
    }
  }
}

void FaultHierarchy::updateSubsystemStatus(std::string name, int status, std::string severity){
  int prev_local_status = m_subsystems[name].locally_faulted;
  if (status == 1){
    m_subsystems[name].current_severity[severity] += 1;
    if (m_subsystems[name].current_severity[severity] >= m_subsystems[name].severity_threshold[severity]){
      m_subsystems[name].locally_faulted = 1;
      m_subsystems[name].status = 1;
    }
  }
  else{
    m_subsystems[name].current_severity[severity] -= 1;
    if (m_subsystems[name].current_severity["Low"] < m_subsystems[name].severity_threshold["Low"]
        && m_subsystems[name].current_severity["Medium"] < m_subsystems[name].severity_threshold["Medium"]
        && m_subsystems[name].current_severity["High"] < m_subsystems[name].severity_threshold["High"]){
      m_subsystems[name].locally_faulted = 0;
      if (m_subsystems[name].hierarchy_faulted == 0){
        m_subsystems[name].status = 0;
      }
    }
  }
  if (prev_local_status != m_subsystems[name].locally_faulted){
    cascadeSubsystemFaults(name, status);
  }
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
        std::cout << "HERE" << i.first << std::endl;
        if (visited.insert(i.first).second && m_subsystems.find(i.first) != m_subsystems.end()){
          // All subfaults need to increment or decrement their hierarchical_fault flag
          if (status){
            m_subsystems[i.first].hierarchy_faulted += 1;
          }
          else{
            m_subsystems[i.first].hierarchy_faulted -= 1;
          }
          // Update subfault status flag based on updated hierarchical_fault flag
          if (m_subsystems[i.first].hierarchy_faulted > 0 || m_subsystems[i.first].locally_faulted == 1){
            m_subsystems[i.first].status = 1;
          }
          else{
            m_subsystems[i.first].status = 0;
          }
          if (i.second == true){
            nodes.push(i.first);
          }
        }
      }
    }
}
