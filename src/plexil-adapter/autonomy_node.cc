// Autonomy ROS node, which is also a PLEXIL application.

// Under construction!

// ROS
#include <ros/ros.h>
#include <ros/package.h>

// OW
#include <owatb_interface/CartesianGuardedMove.h>
#include <ow_lander/StartPlanning.h>
#include "OwAdapter.hh"

// PLEXIL
#include "AdapterFactory.hh" // for REGISTER_ADAPTER() macro
#include "Debug.hh"
#include "Error.hh"
#include "PlexilExec.hh"
#include "ExecApplication.hh"
#include "ExecListenerFactory.hh" // for REGISTER_EXEC_LISTENER() macro
#include "ExecListenerFilterFactory.hh" // for REGISTER_EXEC_LISTENER_FILTER() macro
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

// Forwards
static bool plexil_initialize_interfaces();
static bool initializeExec();

// C++
#include <fstream>
#include <string>
using std::string;
using std::ostringstream;

static ExecApplication* PlexilApp = NULL;

static void test_service_call ()
{
  ros::NodeHandle n;

  ros::ServiceClient client =
    n.serviceClient<ow_lander::StartPlanning>("start_plannning_session");

  if (! client.isValid()) {
    ROS_ERROR("Service client is invalid!");
  }
  else {
    ow_lander::StartPlanning srv;
    srv.request.use_defaults = true;
    srv.request.trench_x = 0.0;
    srv.request.trench_y = 0.0;
    srv.request.trench_d = 0.0;
    srv.request.delete_prev_traj = false;

    if (client.call(srv)) {
      ROS_INFO("StartPlanning returned: %d, %s",
               srv.response.success,
               srv.response.message.c_str());
    }
    else {
      ROS_ERROR("Failed to call service StartPlanning");
    }
  }
}

static void test_plexil_plan ()
{
  string plan = ros::package::getPath("ow_autonomy") +
    "/src/plans/TestStartPlanning.plx";

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
    PlexilApp->step();
    PlexilApp->step();
  }
  catch (const Error& e) {
    ostringstream s;
    s << "Exec error: " << e;
    ROS_ERROR("%s", s.str().c_str());
  }

  delete doc;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "autonomy_node");
  ros::NodeHandle n; // Creating here only for side effects.
  initializeExec();

  // For testing only (works).
  //  test_service_call();

  test_plexil_plan();

  ros::Rate rate(1);
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

static bool plexil_initialize_interfaces()
{
  string config = ros::package::getPath("ow_autonomy") +
    "/src/plans/ow-config.xml";
  const char* config_file = config.c_str();
  pugi::xml_document configDoc;
  pugi::xml_node configElt;
  pugi::xml_parse_result parseResult = configDoc.load_file(config_file);
  if (parseResult.status != pugi::status_ok) {
    ROS_ERROR("Unable to load config file %s: %s",
              config_file,
              parseResult.description());
    return false;
  }
  else {
    configElt = configDoc.child(InterfaceSchema::INTERFACES_TAG());
    if (!configDoc.empty() && configElt.empty()) {
      ROS_ERROR("config file %s has no Interfaces element",
                config_file);
      return false;
    }
  }

  try {
    if (configElt.empty()) {
      // Build default interface configuration if we couldn't load one
      ROS_INFO("Using default interface configuration");
      configElt = configDoc.append_child(InterfaceSchema::INTERFACES_TAG());
    }

    if (!PlexilApp->initialize(configElt)) {
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


static bool initializeExec()
{
  // Throw exceptions, DON'T assert
  Error::doThrowExceptions();
  try {
    REGISTER_ADAPTER(OwAdapter, "Ow");

    PlexilApp = new ExecApplication();
    g_manager = new InterfaceManager(*PlexilApp);

    if (!plexil_initialize_interfaces()) {
      ROS_ERROR("plexil_initialize_interfaces failed");
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
