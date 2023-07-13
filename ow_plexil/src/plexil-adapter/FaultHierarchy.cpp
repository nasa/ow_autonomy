#include "FaultHierarchy.h"

#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <vector>
#include <iostream>
#include "/home/keegan/plexil/src/third-party/pugixml/src/pugixml.hpp"


FaultHierarchy::FaultHierarchy()
{
  std::vector<std::string> test_names = {"fault1", "fault2", "fault3", "fault4", "fault5"};

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
  DebugPrint();
}


void FaultHierarchy::DebugPrint(){
  std::vector<std::string> test_names = {"fault1", "fault2", "fault3", "fault4", "fault5"};
  std::cout << "DEBUG PRINT" << std::endl;
  for (auto i: test_names){
    std::cout << m_fault_model[i].name << 
      " local_fault: " << m_fault_model[i].local_fault <<
      " hierarchical_fault: " << m_fault_model[i].hierarchical_faults <<
      " status: " << m_fault_model[i].status << 
      " severity: " << m_fault_model[i].severity_threshold << " subfaults: ";
    for (auto j: m_fault_model[i].subfaults){
      std::cout << j << " ";
    }
    std::cout << std::endl;
  }
}


void FaultHierarchy::updateFaultModel(const std::string name, const bool status, const int severity){
  std::unordered_set<std::string> visited;
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
