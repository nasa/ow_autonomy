#include "FaultDependencies.h"

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


FaultDependencies::FaultDependencies(const std::string &file_name, bool verbose_flag)
{
  // If verbose we debug print after every update
  m_verbose_flag = verbose_flag;
  const char* source = file_name.c_str();
  // Parse the dependency xml file in plans dir
  if (!parseXML(source)){
    // If we are unable to parse or the file doesnt exist, abort and shutdown node.
    ros::shutdown();
  }
  // Initial print to show setup
  DebugPrint();
}


bool FaultDependencies::checkIsOperable(const std::string &name) const{
  // return true if subsystem/procedure is operable, false if inoperable
  if (m_subsystems.find(name) != m_subsystems.end()){
    return (m_subsystems.at(name).num_inoperable == 0);
  }
  else if (m_procedures.find(name) != m_procedures.end()){
    return (m_procedures.at(name).num_inoperable == 0);
  }
  else{
    ROS_WARN("IsOperable lookup failed, %s does not exist.", name.c_str());
    return true;
  }
}

bool FaultDependencies::checkIsFaulty(const std::string &name) const{
  // checks if subsystem is faulty
  if (m_subsystems.find(name) != m_subsystems.end()){
    return m_subsystems.at(name).faulty;
  }
  else{
    ROS_WARN("IsFaulty lookup failed, %s does not exist.", name.c_str());
    return false;
  }
}


std::vector<std::string> FaultDependencies::getActiveFaults(const std::string &name) const{
  // Limiting it to up to 10 faults returned, this is an arbitrary number to keep 
  // it a fixed size array for PLEXIL.
  std::vector<std::string> active_faults;

  // If System is specified we return all active faults across every subsystem
  if (name == "System"){
    for (auto i: m_faults){
      if (i.second.faulty == 1 && active_faults.size() <= 10){
        active_faults.push_back(i.first);
      }
    }
  }
  // If no subsystem by name exists
  else if(m_subsystems.find(name) == m_subsystems.end()){
    ROS_WARN("Subsystem: %s does not exist.", name.c_str());
  }
  // Get all active faults in specified subsystem
  else{
    for (auto i: m_subsystems.at(name).faults){
      if (m_faults.at(i).faulty == 1 && active_faults.size() <= 10){
        active_faults.push_back(i);
      }
    }
  }
  return active_faults;
}

void FaultDependencies::updateFault(const std::string &name, int status){
  // If a fault doesnt exist
  if (m_faults.find(name) == m_faults.end()){
    return;
  }
  // If a fault exists but does not need to be updated
  if (m_faults[name].faulty == status){
    return;
  }

  m_faults[name].faulty = status;
  std::string parent_subsystem  = m_faults[name].parent_subsystem;


  // increment local fault flag for parent subsystem and change faulty flag as needed
  if (status == 1){
    m_subsystems[parent_subsystem].num_local_faults += 1;
  }
  else{
    m_subsystems[parent_subsystem].num_local_faults -= 1; 
  }

  if (m_subsystems[parent_subsystem].num_local_faults > 0){
    m_subsystems[parent_subsystem].faulty = true; 
  }
  else{
    m_subsystems[parent_subsystem].faulty = false; 
  }

  for (auto dep: m_faults[name].impacts){
    // If an impact to update is a subsystem
    if (dep.second == "Subsystem"){
      if (m_subsystems.find(dep.first) != m_subsystems.end()){
        updateSubsystem(dep.first, status, parent_subsystem);
      }
    }
    // Otherwise we treat it as a procedure to update
    else{
      if (m_procedures.find(dep.first) != m_procedures.end()){
        if (status == 1){
            m_procedures[dep.first].num_inoperable += 1;
        }
        else{
            m_procedures[dep.first].num_inoperable -= 1;
        }
      }
    }
 
  }
  // If verbose output flag is true we print the current fault dependency info
  if (m_verbose_flag){
    DebugPrint();
  }
}

void FaultDependencies::updateSubsystem(const std::string &name, int status, const std::string &parent){
  int prev_inoperable_state = m_subsystems[name].num_inoperable;
  // increment non_local fault and inoperable flags if status is 1
  if (status == 1){
      if (parent != name){
        m_subsystems[name].num_non_local_faults += 1;
      }
      m_subsystems[name].num_inoperable += 1;
      ROS_WARN("Subsystem: %s inoperable.", name.c_str());
  }
  // otherwise decrement
  else{
      if (parent != name){
        m_subsystems[name].num_non_local_faults -= 1;
      }
      m_subsystems[name].num_inoperable -= 1;
      if (m_subsystems[name].num_inoperable == 0){
        ROS_WARN("Subsystem: %s now operable.", name.c_str());
      }
  }
  // If subsystem inoperable flag changes, we need to cascade inoperable fault to impacted subsystems
  if (prev_inoperable_state != m_subsystems[name].num_inoperable){
    cascadeFault(name, status);
  }
}

void FaultDependencies::cascadeFault(const std::string &subsystem_name, int status){
  std::unordered_set<std::string> visited;
  std::stack<std::string> nodes;

  visited.insert(subsystem_name);
  nodes.push(subsystem_name);
  
    // BFS through all subsystem impacts
    while(!nodes.empty()){
      std::string current_node = nodes.top();
      nodes.pop();

      for (auto i: m_subsystems[current_node].impacts){
        // If it is a procedure we do not need to add it to BFS stack, and can just update the inoperable flag
        if (i.second == "Procedure"){
          m_procedures[i.first].num_inoperable = status;
        }
        // Otherwise we update subsystem and add it to stack
        else{
          if (visited.insert(i.first).second && m_subsystems.find(i.first) != m_subsystems.end()){
            // increment or decrement num_non_local_faults flag
            if (status){
              m_subsystems[i.first].num_non_local_faults += 1;
              m_subsystems[i.first].num_inoperable += 1;
            }
            else{
              m_subsystems[i.first].num_non_local_faults -= 1;
              m_subsystems[i.first].num_inoperable -= 1;
            }
            nodes.push(i.first);
          }
        }
      }
    }
}

void FaultDependencies::DebugPrint() const{
  // Prints out current fault info for all faults and subsystems
  ROS_INFO_STREAM("FAULTS: " );
  ROS_INFO_STREAM("------------------------------------------------------------------------"
            << "-----------------------------" 
            );
  ROS_INFO_STREAM(std::setw(40) << std::left << "NAME" << std::setw(20) << "FAULTY");
  ROS_INFO_STREAM("------------------------------------------------------------------------"
            << "-----------------------------" 
            );
  for (auto fault: m_faults){
    ROS_INFO_STREAM(std::left << std::setw(40) << fault.second.name << std::setw(20) << fault.second.faulty );
  }

  ROS_INFO_STREAM("\n" );
  ROS_INFO_STREAM("SUBSYSTEMS/PROCEDURES STATUS" );
  ROS_INFO_STREAM("------------------------------------------------------------------------"
            << "-----------------------------" 
            );

  ROS_INFO_STREAM(std::setw(25) << std::left << "NAME" << std::setw(17) << "TYPE" << std::setw(17) << "NON-LOCAL FAULT" 
  << std::setw(17) << "LOCAL FAULT" << std::setw(17) << "FAULTY" << "INOPERABLE" );
  ROS_INFO_STREAM("------------------------------------------------------------------------"
            << "-----------------------------" 
            );
  for (auto subsystem: m_subsystems){
    ROS_INFO_STREAM(std::left << std::setw(25) << subsystem.second.name << std::setw(17) << "Subsystem" << std::setw(17) << 
    subsystem.second.num_non_local_faults << std::setw(17) << subsystem.second.num_local_faults << std::setw(17) <<
    subsystem.second.faulty << subsystem.second.num_inoperable );
  }
  for (auto proc: m_procedures){
    ROS_INFO_STREAM(std::left << std::setw(25) << proc.second.name << std::setw(17) << "Procedure" << std::setw(17) << 
    "-" << std::setw(17) << "-" << std::setw(17) <<"-" << proc.second.num_inoperable );
  }
}

bool FaultDependencies::parseXML(const std::string &file){
  const char* file_name = file.c_str();
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(file_name);

  // Make sure we can read the xml file, if not print an error message
  if (result)
  {
      ROS_INFO_STREAM("XML [" << file_name << "] parsed without error. \n\n");
  }
  else
  {
      ROS_ERROR_STREAM("XML [" << file_name << "] parsed with errors, attr value: [" << doc.child("node").attribute("attr").value() << "]\n");
      ROS_ERROR_STREAM("Error description: " << result.description() << "\n");
      ROS_ERROR_STREAM("Error offset: " << result.offset << " (error at [..." << (file_name + result.offset) << "]\n\n");
      return false;
  }

  pugi::xml_node subsystems = doc.child("System").child("Subsystems");
  // Read all subsystemes and add them to the m_subsystems map
  for (pugi::xml_node subsystem = subsystems.child("Subsystem"); subsystem; subsystem = subsystem.next_sibling("Subsystem"))
  {
    // get name and severity thresholds
    std::string subsystem_name = subsystem.attribute("Name").as_string();

    // Create and add new subsystem to m_subsystems
    if (m_subsystems.find(subsystem_name) == m_subsystems.end()){
      m_subsystems[subsystem_name] = Subsystem();
      m_subsystems[subsystem_name].name = subsystem_name;
    }
    else{
      continue;
    }
 
    pugi::xml_node faults = subsystem.child("Faults");

    // get all affected subsystems and add them to their respective parent subsystem
    for (pugi::xml_node target = subsystem.child("Impacts"); target;
         target = target.next_sibling("Impacts")){
      if (target.empty()){
        ROS_ERROR_STREAM("Warning: failed to read an impacts target " << target << "\n");
        return false;
      }
      std::string target_name = target.attribute("Name").as_string();
      std::string target_type = target.attribute("Type").as_string();

      m_subsystems[subsystem_name].impacts.push_back(std::make_pair(target_name, target_type));

      if (target_type == "Procedure" && m_procedures.find(target_name) == m_procedures.end()){
        m_procedures[target_name] = Procedure();
        m_procedures[target_name].name = target_name;
      }
    }

    // Read all faults and add them to m_faults as well as their respective subsystems
    for (pugi::xml_node fault = faults.child("Fault"); fault; fault = fault.next_sibling("Fault")){
      std::string fault_name = fault.attribute("Name").as_string();
      if (fault_name.empty()){
        ROS_ERROR_STREAM("Warning: failed to read a fault in subsystem " << subsystem_name << "\n");
        return false;
      }

      if (m_faults.find(fault_name) == m_faults.end()){
        m_faults[fault_name] = Fault();
        m_faults[fault_name].name = fault_name;
        m_faults[fault_name].parent_subsystem = subsystem_name;
        m_subsystems[subsystem_name].faults.push_back(fault_name);
      }
      for (pugi::xml_node impact = fault.child("Impacts"); impact; impact = impact.next_sibling("Impacts")){
        std::string impact_name = impact.attribute("Name").as_string();
        std::string impact_type = impact.attribute("Type").as_string();
        m_faults[fault_name].impacts.push_back(std::make_pair(impact_name, impact_type));
        if (impact_type == "Procedure" && m_procedures.find(impact_name) == m_procedures.end()){
          m_procedures[impact_name] = Procedure();
          m_procedures[impact_name].name = impact_name;
        }
      }
    }
  }
  return true;
}


