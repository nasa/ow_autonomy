// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>

// OW
#include "OwExecutive.h"
#include "OwAdapter.h"

// PLEXIL
#include "AdapterFactory.hh"
#include "AdapterExecInterface.hh"
#include "Debug.hh"
#include "Error.hh"
#include "ExecApplication.hh"
#include "InterfaceSchema.hh"
#include "parsePlan.hh"
#include "State.hh"
using PLEXIL::Error;
using PLEXIL::InterfaceSchema;

// C++
#include <fstream>
#include <string>
#include <iostream>
using std::string;
using std::ostringstream;

// Installed location of compiled PLEXIL files, initialized in initialize().
static string PlexilDir = "";

// The embedded PLEXIL application
static PLEXIL::ExecApplication* PlexilApp = NULL;

OwExecutive* OwExecutive::m_instance = NULL;

OwExecutive* OwExecutive::instance ()
{
  // Very simple singleton
  if (m_instance == NULL) m_instance = new OwExecutive();
  return m_instance;
}

OwExecutive::~OwExecutive()
{
  if (m_instance) delete m_instance;
}

//returns true if current plan is finished executing
bool OwExecutive::getPlanState()
{
	return PlexilApp->allPlansFinished();
}

// stops and resets the exec in case user wants to stop current plan
bool OwExecutive::reset()
{
  if(PlexilApp->stop() == false){ // stop the exec
    ROS_ERROR("Failed to stop the Exec");
    return false; 
  }
  if(PlexilApp->reset() == false){ // reset the exec
    ROS_ERROR("Failed to reset the Exec");
    return false; 
  }
  if (!PlexilApp->step()) {
      ROS_ERROR("Stepping exec failed");
      return false;
  }

  return true; // returns true if both were succesful
}

bool OwExecutive::runPlan (const string& filename)
{
  string plan = (PlexilDir + filename);

  pugi::xml_document* doc = NULL;
  try {
    doc = PLEXIL::loadXmlFile (plan);
  }
  catch (PLEXIL::ParserException const &e) {
    ROS_ERROR("Load of PLEXIL plan %s failed: %s", plan.c_str(), e.what());
    return false;
  }

  if (!doc) {
    ROS_ERROR("PLEXIL plan %s not found", plan.c_str());
    return false;
  }

  try {
    PlexilApp->addPlan (doc);
  }
  catch (PLEXIL::ParserException const &e) {
    ROS_ERROR("Add of PLEXIL plan %s failed: %s", plan.c_str(), e.what());
    return false;
  }

  try {
	// updates Exec so that multiple plans can be run even after first plan finishes 
  PlexilApp->notifyExec();
	g_execInterface->handleValueChange(PLEXIL::State::timeState(), 0);
  PlexilApp->run();
  }
  catch (const Error& e) {
    ostringstream s;
    s << "Exec error: " << e;
    ROS_ERROR("%s", s.str().c_str());
    return false;
  }

  delete doc;
  return true;
}


// PLEXIL application setup functions start here.

static bool plexilInitializeInterfaces()
{
  string config = (PlexilDir + "ow-config.xml");
  const char* config_file = config.c_str();
  pugi::xml_document config_doc;
  pugi::xml_node config_elt;
  pugi::xml_parse_result parseResult = config_doc.load_file(config_file);
  if (parseResult.status != pugi::status_ok) {
    ROS_ERROR("Unable to load config file %s: %s",
              config_file,
              parseResult.description());
    return false;
  }
  else {
    config_elt = config_doc.child(InterfaceSchema::INTERFACES_TAG());
    if (!config_doc.empty() && config_elt.empty()) {
      ROS_ERROR("config file %s has no Interfaces element", config_file);
      return false;
    }
  }

  try {
    if (config_elt.empty()) {
      // Build default interface configuration if we couldn't load one
      ROS_INFO("Using default interface configuration");
      config_elt = config_doc.append_child(InterfaceSchema::INTERFACES_TAG());
    }

    if (!PlexilApp->initialize(config_elt)) {
      ROS_ERROR("Interface initialization failed");
      return false;
    }
  }
  catch (const Error& e) {
    ostringstream s;
    s << "Exec init error: " << e;
    ROS_ERROR("%s", s.str().c_str());
    return false;
  }
  return true;
}

static void get_plexil_debug_config()
{
  try {
    string debug_file = PlexilDir + "plexil-debug.cfg";
    std::ifstream dbgConfig(debug_file.c_str());
    if (dbgConfig.good()) PLEXIL::readDebugConfigStream(dbgConfig);
    else ROS_ERROR("Unable to open PLEXIL debug config file %s",
                   debug_file.c_str());
  }
  catch (const Error& e) {
    std::ostringstream s;
    e.print(s);
    ROS_ERROR("Error getting PLEXIL debug config: %s", s.str().c_str());
  }
}

bool OwExecutive::initialize ()
{
  // NOTE: this is the best we can do for now.
  // ROS provides no API to locate the 'devel' directory.
  PlexilDir = ros::package::getPath("ow_plexil") + "/../../../devel/etc/plexil/";

  // Throw exceptions, DON'T assert
  Error::doThrowExceptions();

  get_plexil_debug_config();

  try {
    REGISTER_ADAPTER(OwAdapter, "Ow");
    PlexilApp = new PLEXIL::ExecApplication();
    if (!plexilInitializeInterfaces()) {
      ROS_ERROR("plexilInitializeInterfaces failed");
      return false;
    }
    if (!PlexilApp->startInterfaces()) {
      ROS_ERROR("Interface startup failed");
      return false;
    }
    if (!PlexilApp->step()) {
      ROS_ERROR("Stepping exec failed");
      return false;
    }
    PlexilApp->addLibraryPath (PlexilDir);
  }
  catch (const Error& e) {
    ostringstream s;
    s << "Exec init error: " << e;
    ROS_ERROR("%s", s.str().c_str());
    return false;
  }
  return true;
}
