// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>

// OW
#include "OwExecutive.h"
#include "OwAdapter.h"

// PLEXIL
#include "AdapterFactory.hh"
#include "Debug.hh"
#include "Error.hh"
#include "PlexilExec.hh"
#include "ExecApplication.hh"
#include "InterfaceManager.hh"
#include "InterfaceSchema.hh"
#include "parsePlan.hh"
#include "State.hh"
using PLEXIL::Error;
using PLEXIL::ExecApplication;
using PLEXIL::InterfaceManager;
using PLEXIL::InterfaceSchema;
using PLEXIL::State;
using PLEXIL::Value;

// C++
#include <fstream>
#include <string>
#include <iostream>
using std::string;
using std::ostringstream;
using std::cout;
using std::endl;

// Location of PLEXIL files
static string PlexilDir = "";  // initialized below

// The embedded PLEXIL application
static ExecApplication* PlexilApp = NULL;

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
    g_manager->handleAddPlan(doc->document_element());
  }
  catch (PLEXIL::ParserException const &e) {
    ROS_ERROR("Add of PLEXIL plan %s failed: %s", plan.c_str(), e.what());
    return false;
  }

  try {
    g_manager->handleValueChange(State::timeState(), 0);
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


bool OwExecutive::initialize ()
{
  // NOTE: this is the best we can do for now.  Eventually these would be in the
  // install space.
  PlexilDir = ros::package::getPath("ow_autonomy") + "/../../devel/etc/plexil/";

  // Throw exceptions, DON'T assert
  Error::doThrowExceptions();

  try {
    REGISTER_ADAPTER(OwAdapter, "Ow");

    PlexilApp = new ExecApplication();
    g_manager = new InterfaceManager(*PlexilApp);

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

// End of PLEXIL application setup functions.
