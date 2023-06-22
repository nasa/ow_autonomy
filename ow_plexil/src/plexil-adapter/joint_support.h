// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef OW_AUTONOMY_JOINT_SUPPORT_H
#define OW_AUTONOMY_JOINT_SUPPORT_H

// Support for lander joints.

// NOTE: Most of this file supports OceanWATERS, and only a bit is
// used in OWLAT.  Only OceanWATERS uses ROS's /joint_states topic,
// and relies on its joint ordering, which is assumed fixed.

#include <string>

const size_t NumJoints = 9; // arm + antenna
const size_t NumArmJoints = 7;

enum Joint {
  // This enumeration must list all joints and in the same order as
  // the topic /joint_states, which is alphabetical by joint name.
  // The names used here are different but more readable.
  ANTENNA_PAN = 0,
  ANTENNA_TILT = 1,
  DISTAL_PITCH = 2,
  GRINDER = 3,
  HAND_YAW = 4,
  PROXIMAL_PITCH = 5,
  SCOOP_YAW = 6,
  SHOULDER_PITCH = 7,
  SHOULDER_YAW = 8
};

struct JointProperties
{
  // A structure to store useful static properties about joints.

  // Use compiler's default methods.
  std::string plexilName; // human-readable, no spaces
  double softTorqueLimit;
  double hardTorqueLimit;
};

struct JointTelemetry
{
  // Structure to store the latest joint telemetry readings.

  // Use compiler's default methods.

  double position = 0;
  double velocity = 0;
  double effort = 0;
  double acceleration = 0;
};

#endif
