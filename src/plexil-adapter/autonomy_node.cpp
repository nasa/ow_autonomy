// Autonomy ROS node, which embeds a PLEXIL application.

// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

// Under construction!  The current version is skeletal and hardcodes some basic
// concept proofs.

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <owatb_interface/CartesianGuardedMove.h>
#include <ow_lander/StartPlanning.h>
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
using std::string;
using std::ostringstream;

// The embedded PLEXIL application
static ExecApplication* PlexilApp = NULL;

static void test_plexil_plan ()
{
  // Temporary function, a test of a specific plan.

  string plan = ros::package::getPath("ow_autonomy") +
    // Hack! This is not where we want to look:
    // "/src/plans/TestOwLander.plx";
    // But this is, and I don't know a better way to specify this directory:
    "/../../devel/etc/plexil/TestOwLander.plx";

  pugi::xml_document* doc = NULL;
  try {
    doc = PLEXIL::loadXmlFile (plan);
  }
  catch (PLEXIL::ParserException const &e) {
    ROS_ERROR("Load of PLEXIL plan %s failed: %s",
              plan.c_str(),
              e.what());
    return;
  }

  if (!doc) {
    ROS_ERROR("PLEXIL plan %s not found", plan.c_str());
    return;
  }

  try {
    g_manager->handleAddPlan(doc->document_element());
  }
  catch (PLEXIL::ParserException const &e) {
    ROS_ERROR("Add of PLEXIL plan %s failed: %s",
              plan.c_str(),
              e.what());
    return;
  }

  try {
    g_manager->handleValueChange(State::timeState(), 0);
    PlexilApp->run();
  }
  catch (const Error& e) {
    ostringstream s;
    s << "Exec error: " << e;
    ROS_ERROR("%s", s.str().c_str());
  }

  delete doc;
}


// PLEXIL application setup functions start here.

static bool plexilInitializeInterfaces()
{
  string config =
    // Hack! This is not where we want to look:
    //    ros::package::getPath("ow_autonomy") + "/src/plans/ow-config.xml";
    // But this is, and I don't know a better way to specify this directory
    ros::package::getPath("ow_autonomy") + "/../../devel/etc/plexil/ow-config.xml";
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


static bool initialize_exec()
{
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

    // Later.  No libraries yet
    //    if (!plexil_initialize_library_path())
    //      return false;
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


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "autonomy_node");
  ros::NodeHandle node_handle; // Not used yet. Created to start node.

  initialize_exec();

  // This is just a test, a proof of concept.  The general functionality of this
  // node needs design and implementation.
  test_plexil_plan();

  ros::Rate rate(1);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
