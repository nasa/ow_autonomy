// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>

// OW
#include "OwExecutive.h"

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
#include <stdlib.h>
#include <fstream>
#include <string>
#include <iostream>
using std::string;
using std::ostringstream;

// Installed location of compiled PLEXIL files, initialized in initialize().
static string PlexilDir = "";

// The embedded PLEXIL application
static PLEXIL::ExecApplication* PlexilApp = NULL;

OwExecutive* OwExecutive::instance ()
{
  // Very simple singleton
  static OwExecutive instance;
  return &instance;
}

bool OwExecutive::getPlanState()
{
  return PlexilApp->allPlansFinished();
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
    PLEXIL::g_execInterface->handleValueChange(PLEXIL::State::timeState(), 0);
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

static bool plexilInitializeInterfaces (const string& config_file)
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
    config_elt = config_doc.child(InterfaceSchema::INTERFACES_TAG());
    if (!config_doc.empty() && config_elt.empty()) {
      ROS_ERROR("config file %s has no Interfaces element", config_path.c_str());
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
    PlexilApp = new PLEXIL::ExecApplication();
    if (! plexilInitializeInterfaces (config_file)) {
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
