#include "FaultHierarchy.h"

#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <vector>
#include <iostream>
//#include "/home/keegan/plexil/src/third-party/pugixml/src/pugixml.hpp"
#include "pugixml.hpp"


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
  //DebugPrint();
  //
  const char* source = "xmltest.xml";
  pugi::xml_document doc;
  pugi::xml_parse_result result = doc.load_file(source);

  if (result)
  {
      std::cout << "XML [" << source << "] parsed without errors, attr value: [" << doc.child("node").attribute("attr").value() << "]\n\n";
  }
  else
  {
      std::cout << "XML [" << source << "] parsed with errors, attr value: [" << doc.child("node").attribute("attr").value() << "]\n";
      std::cout << "Error description: " << result.description() << "\n";
      std::cout << "Error offset: " << result.offset << " (error at [..." << (source + result.offset) << "]\n\n";
  }


  pugi::xml_node subsystems = doc.child("System").child("Subsystems");
  for (pugi::xml_node subsystem = subsystems.child("Subsystem"); subsystem; subsystem = subsystem.next_sibling("Subsystem"))
  {
    std::cout << "Subsystem " << subsystem.attribute("Name").value() << "\n";
    for (pugi::xml_node fault_group = subsystem.child("FaultGroup"); fault_group; fault_group = fault_group.next_sibling("FaultGroup")){
      std::cout << "FaultGroup " << fault_group.attribute("Name").value() << "\n";
      pugi::xml_node faults = fault_group.child("Faults");
      pugi::xml_node affected_subsystems = fault_group.child("AffectedSubsystems");
      for (pugi::xml_node fault = faults.child("Fault"); fault; fault = fault.next_sibling("Fault")){
        std::cout << "Fault " << fault.attribute("Name").value() << "\n";
      }
      for (pugi::xml_node affected_subsystem = affected_subsystems.child("Sys"); affected_subsystem;
           affected_subsystem = affected_subsystem.next_sibling("Sys")){
        std::cout << "AffectedSubsystems " << affected_subsystem.attribute("Name").value() << "\n";
      }
 
    }
  }
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
