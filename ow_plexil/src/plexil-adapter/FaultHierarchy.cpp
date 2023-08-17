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


bool FaultHierarchy::checkIsOperable(std::string name){
  if (m_subsystems.find(name) != m_subsystems.end()){
    return !m_subsystems[name].inoperable;
  }
  else if (m_procedures.find(name) != m_procedures.end()){
    return !m_procedures[name].inoperable;
  }
  else{
    ROS_WARN("IsOperable LOOKUP FAILED, %s DOES NOT EXIST.", name.c_str());
    return true;
  }
}

bool FaultHierarchy::checkIsFaulty(std::string name){
  if (m_subsystems.find(name) != m_subsystems.end()){
    return m_subsystems[name].faulty;
  }
  else{
    ROS_WARN("IsFaulty LOOKUP FAILED, %s DOES NOT EXIST.", name.c_str());
    return false;
  }
}


std::vector<std::string> FaultHierarchy::getActiveFaults(std::string name){
  //LIMITING TO MAX OF 10 FAULTS RETURNED DUE TO PLEXIL RESTRICTIONS
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
    ROS_WARN("SUBSYSTEM: %s DOES NOT EXIST.", name.c_str());
  }
  // Get all active faults in specified subsystem
  else{
    for (auto i: m_subsystems[name].faults){
      if (m_faults[i].faulty == 1 && active_faults.size() <= 10){
        active_faults.push_back(i);
      }
    }
  }
  return active_faults;
}

void FaultHierarchy::updateFault(std::string name, int status){
  // If a fault doesnt exist
  if (m_faults.find(name) == m_faults.end()){
    ROS_WARN("FAULT NAME: %s DOES NOT EXIST.", name.c_str());
    return;
  }
  // If a fault exists but does not need to be updated
  if (m_faults[name].faulty == status){
    return;
  }

  m_faults[name].faulty = status;

  for (auto dep: m_faults[name].dependencies){
    // If a dependency to update is a subsystem
    if (dep.second == "Subsystem"){
      if (m_subsystems.find(dep.first) != m_subsystems.end()){
        updateSubsystem(dep.first, status);
      }
    }
    // Otherwise we treat it as a procedure to update
    else{
      if (m_procedures.find(dep.first) != m_procedures.end()){
        m_procedures[name].inoperable = status;
      }
    }
 
  }
  // If verbose output flag is true we print the current fault hierarchy info
  if (m_verbose_flag){
    DebugPrint();
  }
}

void FaultHierarchy::updateSubsystem(std::string name, int status){
  int prev_faulty_state = m_subsystems[name].faulty;
  if (status == 1){
    // If the current severity is over the threshold we locally fault and change the status
      m_subsystems[name].local_fault += 1;
      m_subsystems[name].faulty = 1;
      m_subsystems[name].inoperable = 1;
      ROS_INFO("SUBSYSTEM: %s FAULTY, LOCAL FAULT.", name.c_str());
  }
  else{
    m_subsystems[name].local_fault -= 1;

    if (m_subsystems[name].local_fault == 0){
      m_subsystems[name].faulty = 0;
    }

    if (m_subsystems[name].non_local_fault == 0 && m_subsystems[name].faulty == 0){
      m_subsystems[name].inoperable = 0;
      ROS_INFO("SUBSYSTEM: %s FAULT CLEARED.", name.c_str());
    }
  }
  // If we locally faulted, we need to cascade a hierarchy fault to depended/affected subsystems
  if (prev_faulty_state != m_subsystems[name].faulty){
    cascadeFault(name, status);
  }
}

void FaultHierarchy::cascadeFault(std::string subsystem_name, int status){
  std::unordered_set<std::string> visited;
  std::stack<std::string> nodes;

  visited.insert(subsystem_name);
  nodes.push(subsystem_name);
  
    // BFS through all subsystem dependencies
    while(!nodes.empty()){
      std::string current_node = nodes.top();
      nodes.pop();

      for (auto i: m_subsystems[current_node].dependencies){
        // If it is a procedure we do not need to add it to BFS stack, and can just update the inoperable flag
        if (i.second == "Procedure"){
          m_procedures[i.first].inoperable = status;
        }
        // Otherwise we update subsystem and add it to stack
        else{
          if (visited.insert(i.first).second && m_subsystems.find(i.first) != m_subsystems.end()){
            // increment or decrement non_local_fault flag
            if (status){
              m_subsystems[i.first].non_local_fault += 1;
            }
            else{
              m_subsystems[i.first].non_local_fault -= 1;
            }
            // Update subsystem flags based on current local and non_local fault flags
            if (m_subsystems[i.first].non_local_fault > 0 || m_subsystems[i.first].local_fault > 0){
              m_subsystems[i.first].inoperable = 1;
            }
            else{
              m_subsystems[i.first].inoperable = 0;
              ROS_INFO("SUBSYSTEM: %s OPERABLE, ALL NON-LOCAL FAULTS CLEARED.", i.first.c_str());
            }
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
  std::cout << std::setw(40) << std::left << "NAME" << std::setw(20) << "FAULTY" << std::setw(27) << "DEPENDENCIES" 
  <<  std::endl;
  std::cout << "------------------------------------------------------------------------"
            << "------------------------------------------------------------------------" 
            << std::endl;
  for (auto fault: m_faults){
    std::cout << std::left << std::setw(40) << fault.second.name << std::setw(20) << fault.second.faulty;
    for (auto i: m_faults[fault.second.name].dependencies){
      std::cout << std::setw(1) << "(" << i.first << ", " << i.second << ") ";
    }
    std::cout << std::endl;
  }

  std::cout << "\n" << std::endl;
  std::cout << "SUBSYSTEMS/PROCEDURES STATUS" << std::endl;
  std::cout << "------------------------------------------------------------------------"
            << "------------------------------------------------------------------------" 
            << std::endl;

  std::cout << std::setw(20) << std::left << "NAME" << std::setw(20) << "TYPE" << std::setw(27) << "NON-LOCAL FAULT" 
  << std::setw(20) << "LOCAL FAULT" << std::setw(20) << "FAULTY" << std::setw(20) << "INOPERABLE"<<
  std::setw(10) << "DEPENDENCIES" << std::endl; 
  std::cout << "------------------------------------------------------------------------"
            << "------------------------------------------------------------------------" 
            << std::endl;
  for (auto subsystem: m_subsystems){
    std::cout << std::left << std::setw(20) << subsystem.second.name << std::setw(20) << "Subsystem" << std::setw(27) << 
    subsystem.second.non_local_fault << std::setw(20) << subsystem.second.local_fault <<
    subsystem.second.faulty << std::setw(20) << subsystem.second.inoperable;
    for (auto i: m_subsystems[subsystem.second.name].dependencies){
      std::cout << i.first << " ";
    }
    std::cout << std::endl;
  }
  for (auto proc: m_procedures){
    std::cout << std::left << std::setw(20) << proc.second.name << std::setw(20) << "Procedure" << std::setw(27) << 
    "-" << std::setw(20) << "-" << "-" << std::setw(20) << proc.second.inoperable;
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
      std::cout << "XML [" << file_name << "] parsed without error. \n\n";
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
    for (pugi::xml_node target = subsystem.child("Disables"); target;
         target = target.next_sibling("Disables")){
      if (target.empty()){
        std::cout << "WARNING: FAILED TO READ A DISABLE TARGET " << target << "\n";
        continue;
      }
      std::string target_name = target.attribute("Name").value();
      std::string target_type = target.attribute("Type").value();

      m_subsystems[subsystem_name].dependencies.push_back(std::make_pair(target_name, target_type));

      if (target_type == "Procedure" && m_procedures.find(target_name) == m_procedures.end()){
        m_procedures[target_name] = Procedure();
        m_procedures[target_name].name = target_name;
      }
    }

    // Read all faults and add them to m_faults as well as their respective subsystems
    for (pugi::xml_node fault = faults.child("Fault"); fault; fault = fault.next_sibling("Fault")){
      std::string fault_name = fault.attribute("Name").value();
      if (fault_name.empty()){
        std::cout << "WARNING: FAILED TO READ A FAULT IN SUBSYSTEM " << subsystem_name << "\n";
        continue;
      }

      if (m_faults.find(fault_name) == m_faults.end()){
        m_faults[fault_name] = Fault();
        m_faults[fault_name].name = fault_name;
      }
      m_subsystems[subsystem_name].faults.push_back(fault_name);
      for (pugi::xml_node dep = fault.child("Disables"); dep; dep = dep.next_sibling("Disables")){
        std::string dependency_name = dep.attribute("Name").value();
        std::string dependency_type = dep.attribute("Type").value();
        m_faults[fault_name].dependencies.push_back(std::make_pair(dependency_name, dependency_type));
        if (dependency_type == "Procedure" && m_procedures.find(dependency_name) == m_procedures.end()){
          m_procedures[dependency_name] = Procedure();
          m_procedures[dependency_name].name = dependency_name;
        }
      }
    }
  }
}


