// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef OW_AUTONOMY_JOINT_SUPPORT_H
#define OW_AUTONOMY_JOINT_SUPPORT_H

// Support for lander joints, based on ROS /joint_states message.

// NOTE: This package (ow_plexil) is implemented for the lander in the
// ow_simulator package.  Specifically, the joints and joint ordering
// (in /joint_states) is assumed fixed, and is often hardcoded.

#include <string>

const size_t NumJoints = 9;

enum Joint {
  // This enumeration must list all joints and in the same order as
  // the topic /joint_states, which is alphabetical by joint name
  // there.  The names used here are different but more readable.
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

  JointTelemetry (double p = 0, double v = 0, double e = 0, double a = 0)
  : position(p),
    velocity(v),
    effort(e),
    acceleration(a)
  { }

  // Use compiler's copy constructor, destructor, assignment.

  double position;
  double velocity;
  double effort;
  double acceleration;
};

enum class TelemetryType {
  Position,
  Velocity,
  Effort,
  Acceleration
};

#endif
