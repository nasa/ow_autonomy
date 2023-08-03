#include "FaultHierarchy.h"

// ROS
#include <ros/ros.h>

// C++
#include <unordered_map>
#include <iomanip>      // std::setw
#include <unordered_set>
#include <stack>
#include <vector>
#include <iostream>
#include <memory>

// PUGIXML
#include "pugixml.hpp"


FaultHierarchy::FaultHierarchy(std::string file_name, bool verbose_flag)
{
  // If verbose we debug print after every update
  m_verbose_flag = verbose_flag;
  const char* source = file_name.c_str();
  // Parse the hierarchy xml file in plans dir
  parseXML(source);
  // Initial print to show setup
  DebugPrint();
}

std::vector<std::string> FaultHierarchy::getActiveFaults(std::string name){
  //LIMITING TO MAX OF 10 FAULTS RETURNED DUE TO PLEXIL RESTRICTIONS
  std::vector<std::string> active_faults;

  // If System is specified we return all active faults across every subsystem
  if (name == "System"){
    for (auto i: m_faults){
      if (i.second.status == 1 && active_faults.size() <= 10){
        active_faults.push_back(i.first);
      }
    }
  }
  // If no subsystem by name exists
  else if(m_subsystems.find(name) == m_subsystems.end()){
    ROS_WARN("SUBSYSTEM: %s DOES NOT EXIST.", name.c_str());
  }
  // Get all active faults in specified subsystem
  else{
    for (auto i: m_subsystems[name].faults){
      if (m_faults[i].status == 1 && active_faults.size() <= 10){
        active_faults.push_back(i);
      }
    }
  }
  return active_faults;
}

std::vector<bool> FaultHierarchy::getSubsystemStatus(std::string name){
  // return the subsytem status if it exists
  std::vector<bool> status(3,0);
  if (m_subsystems.find(name) == m_subsystems.end()){
    ROS_WARN("SUBSYSTEM: %s DOES NOT EXIST.", name.c_str());
    return status;
  }
  // 3 integers to represent status -> [STATUS, LOCALLY FAULTED, HIERARCHICALLY FAULTED]
  status[0] = m_subsystems[name].status;
  status[1] = m_subsystems[name].locally_faulted;
  status[2] = m_subsystems[name].hierarchy_faulted;
  return status;
}

void FaultHierarchy::updateFaultHierarchy(std::string name, int status){
  // If a fault doesnt exist
  if (m_faults.find(name) == m_faults.end()){
    ROS_WARN("FAULT NAME: %s DOES NOT EXIST.", name.c_str());
    return;
  }
  // If a fault exists but does not need to be updated
  if (m_faults[name].status == status){
    return;
  }
  // For every subsystem that a fault affects, we check if the subsystem needs to be updated
  m_faults[name].status = status;
  for (auto affected_sub: m_faults[name].affected_subsystems){
    if (m_subsystems.find(affected_sub.first) != m_subsystems.end()){
      updateSubsystemStatus(affected_sub.first, status, affected_sub.second);
    }
  }
  // If verbose output flag is true we print the current fault hierarchy info
  if (m_verbose_flag){
    DebugPrint();
  }
}

void FaultHierarchy::updateSubsystemStatus(std::string name, int status, std::string severity){
  int prev_local_status = m_subsystems[name].locally_faulted;
  if (status == 1){
    // If a fault is triggered we increment the current severity level of that subsystem
    m_subsystems[name].current_severity[severity] += 1;
    // If the current severity is over the threshold we locally fault and change the status
    if (m_subsystems[name].current_severity[severity] >= m_subsystems[name].severity_threshold[severity]){
      m_subsystems[name].locally_faulted = 1;
      m_subsystems[name].status = 1;
      ROS_INFO("SUBSYSTEM: %s LOCALLY FAULTED.", name.c_str());
    }
  }
  else{
    // If a fault cleared  we decrement the current severity level of that subsystem
    m_subsystems[name].current_severity[severity] -= 1;
    // If all severyity levels are below the threshold we clear the local fault flag of the subsystem
    if (m_subsystems[name].current_severity["Low"] < m_subsystems[name].severity_threshold["Low"]
        && m_subsystems[name].current_severity["Medium"] < m_subsystems[name].severity_threshold["Medium"]
        && m_subsystems[name].current_severity["High"] < m_subsystems[name].severity_threshold["High"]){
      m_subsystems[name].locally_faulted = 0;
      // if the hierarchy fault is also clear, the status is changed back to 0
      if (m_subsystems[name].hierarchy_faulted == 0 && m_subsystems[name].status == 1){
        m_subsystems[name].status = 0;
        ROS_INFO("SUBSYSTEM: %s NO LONGER FAULTED.", name.c_str());
      }
    }
  }
  // If we locally faulted, we need to cascade a hierarchy fault to depended/affected subsystems
  if (prev_local_status != m_subsystems[name].locally_faulted){
    cascadeSubsystemFaults(name, status);
  }
}


void FaultHierarchy::cascadeSubsystemFaults(std::string subsystem_name, int status){
  std::unordered_set<std::string> visited;
  std::stack<std::string> nodes;

  visited.insert(subsystem_name);
  nodes.push(subsystem_name);
  
    // BFS through all subsystem dependencies
    while(!nodes.empty()){
      std::string current_node = nodes.top();
      nodes.pop();

      for (auto i: m_subsystems[current_node].affected_subsystems){
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
            ROS_INFO("SUBSYSTEM: %s HIERARCHY FAULTED.", i.first.c_str());
          }
          else{
            m_subsystems[i.first].status = 0;
            ROS_INFO("SUBSYSTEM: %s NO LONGER HIERARCHY FAULTED.", i.first.c_str());
          }
          if (i.second == true){
            nodes.push(i.first);
          }
        }
      }
    }
}

void FaultHierarchy::DebugPrint(){
  // Prints out current hierarchical fault info for all faults and subsystems
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

void FaultHierarchy::parseXML(const char* file_name){
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(file_name);

  // Make sure we can read the xml file, if not print an error message
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
  // Read all subsystemes and add them to the m_subsystems map
  for (pugi::xml_node subsystem = subsystems.child("Subsystem"); subsystem; subsystem = subsystem.next_sibling("Subsystem"))
  {
    // get name and severity thresholds
    std::string subsystem_name = subsystem.attribute("Name").value();
    std::string subsystem_severity_low = subsystem.child("SeverityThreshold").attribute("Low").value();
    std::string subsystem_severity_med = subsystem.child("SeverityThreshold").attribute("Medium").value();
    std::string subsystem_severity_high = subsystem.child("SeverityThreshold").attribute("High").value();

    // Make sure the info is present in the XML
    if (subsystem_name.empty() ||  subsystem_severity_low.empty() || subsystem_severity_med.empty()
        || subsystem_severity_high.empty() ){
      std::cout << "WARNING: FAILED TO READ A SUBSYSTEM " << subsystem_name << "\n";
      continue;
    }

    // Create and add new subsystem to m_subsystems
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
 
    pugi::xml_node sys_affected_subsystems = subsystem.child("AffectedSubsystems");
    pugi::xml_node faults = subsystem.child("Faults");

    // get all affected subsystems and add them to their respective parent subsystem
    for (pugi::xml_node affected_subsystem = sys_affected_subsystems.child("Sys"); affected_subsystem;
         affected_subsystem = affected_subsystem.next_sibling("Sys")){
      if (affected_subsystem.empty()){
        std::cout << "WARNING: FAILED TO READ AN AFFECTED SUBSYSTEM " << affected_subsystem << "\n";
        continue;
      }
      std::string affected_subsystem_name = affected_subsystem.attribute("Name").value();
      std::string affected_subsystem_cascade_flag_str = affected_subsystem.attribute("Cascade").value();
      bool cascade_flag = false;
      // get cascade flag
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
    }

    // Read all faults and add them to m_faults as well as their respective subsystems
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
    }
  }
}


