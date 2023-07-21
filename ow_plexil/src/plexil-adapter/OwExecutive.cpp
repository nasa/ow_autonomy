// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// C++
#include <stdlib.h>
#include <fstream>
#include <string>
#include <iostream>
using std::string;
using std::ostringstream;

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>

// PLEXIL
#include <AdapterFactory.hh>
#include <AdapterExecInterface.hh>
#include <Debug.hh>
#include <Error.hh>
#include <InterfaceSchema.hh>
#include <parsePlan.hh>
#include <State.hh>
using namespace PLEXIL;

// OW
#include "OwExecutive.h"

// Installed location of compiled PLEXIL files, initialized in initialize().
static string PlexilDir = "";

OwExecutive::OwExecutive()
  : m_plexil_app (makeExecApplication())
{
}

OwExecutive* OwExecutive::instance ()
{
  // Very simple singleton
  static OwExecutive instance;
  return &instance;
}

bool OwExecutive::allPlansFinished()
{
  // WARNING: The called function returns true iff the PLEXIL
  // executive has executed at least one plan, and no plans are
  // currently running.  This is NOT accurate enough for the callers
  // of this method.  The PLEXIL team has been consulted for a better
  // approach on 12/16/22.
  return m_plexil_app->allPlansFinished();
}

bool OwExecutive::runPlan (const string& filename)
{
  string plan = (PlexilDir + filename);

  pugi::xml_document* doc = NULL;
  try {
    doc = loadXmlFile (plan);
  }
  catch (ParserException const &e) {
    ROS_ERROR("Load of PLEXIL plan %s failed: %s", plan.c_str(), e.what());
    return false;
  }

  if (!doc) {
    ROS_ERROR("PLEXIL plan %s not found", plan.c_str());
    return false;
  }

  try {
    m_plexil_app->addPlan (doc);
  }
  catch (ParserException const &e) {
    ROS_ERROR("Add of PLEXIL plan %s failed: %s", plan.c_str(), e.what());
    return false;
  }

  try {
    // Updates Exec so that multiple plans can be run even after first
    // plan finishes.
    m_plexil_app->notifyExec();
    OwExecutive::instance()
      -> plexilInterfaceMgr()
      -> handleValueChange(State::timeState(), 0);
    if (! m_plexil_app->run()) {
      ROS_ERROR ("Running the PLEXIL application failed.");
      return false;
    }
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

bool OwExecutive::initializePlexilInterfaces (const string& config_file)
{
  string config_path = (PlexilDir + config_file);
  pugi::xml_document config_doc;
  pugi::xml_node config_elt;
  pugi::xml_parse_result parse_result = config_doc.load_file (config_path.c_str());
  if (parse_result.status != pugi::status_ok) {
    ROS_ERROR("Unable to load config file %s: %s",
              config_path.c_str(),
              parse_result.description());
    return false;
  }
  else {
    config_elt = config_doc.child(InterfaceSchema::INTERFACES_TAG);
    if (!config_doc.empty() && config_elt.empty()) {
      ROS_ERROR("config file %s has no Interfaces element", config_path.c_str());
      return false;
    }
  }

  try {
    if (config_elt.empty()) {
      // Build default interface configuration if we couldn't load one
      ROS_INFO("Using default interface configuration");
      config_elt = config_doc.append_child(InterfaceSchema::INTERFACES_TAG);
    }

    if (!m_plexil_app->initialize(config_elt)) {
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
    if (dbgConfig.good()) readDebugConfigStream(dbgConfig);
    else ROS_ERROR("Unable to open PLEXIL debug config file %s",
                   debug_file.c_str());
  }
  catch (const Error& e) {
    std::ostringstream s;
    e.print(s);
    ROS_ERROR("Error getting PLEXIL debug config: %s", s.str().c_str());
  }
}

bool OwExecutive::initialize (const string& config_file)
{
  // Get plan library directory from the environment variable set by
  // the ow_plexil env-hooks
  char *plexil_plan_dir_env = getenv("PLEXIL_PLAN_DIR");
  if(plexil_plan_dir_env == NULL) {
    ROS_ERROR("Environment variable $PLEXIL_PLAN_DIR is not set.");
    return false;
  }

  PlexilDir = plexil_plan_dir_env+string("/");

  // Throw exceptions, DON'T assert
  Error::doThrowExceptions();

  get_plexil_debug_config();

  try {
    m_plexil_app = makeExecApplication();
    if (! initializePlexilInterfaces (config_file)) {
      ROS_ERROR("Failed to initialize PLEXIL interfaces.");
      return false;
    }
    if (!m_plexil_app->startInterfaces()) {
      ROS_ERROR("PLEXIL interface startup failed");
      return false;
    }
    if (!m_plexil_app->step()) {
      ROS_ERROR("Stepping exec failed");
      return false;
    }
    m_plexil_app->addLibraryPath (PlexilDir);
  }
  catch (const Error& e) {
    ostringstream s;
    s << "Exec init error: " << e;
    ROS_ERROR("%s", s.str().c_str());
    return false;
  }
  return true;
}

InterfaceManager* OwExecutive::plexilInterfaceMgr()
{
  return m_plexil_app->manager();
}

AdapterConfiguration* OwExecutive::plexilAdapterConfig()
{
  return m_plexil_app->configuration();
}
