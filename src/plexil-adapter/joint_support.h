// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef OW_AUTONOMY_JOINT_SUPPORT_H
#define OW_AUTONOMY_JOINT_SUPPORT_H

// Support for lander joints, based on ROS /joint_states message.

#include <string>

enum class Joint {
  shoulder_yaw,
  shoulder_pitch,
  proximal_pitch,
  distal_pitch,
  hand_yaw,
  scoop_yaw,
  antenna_pan,
  antenna_tilt,
  grinder
};

struct JointProperties
{
  // Use compiler's default methods.
  std::string rosName;
  std::string plexilName; // human-readable, no spaces
  double softTorqueLimit;
  double hardTorqueLimit;
};

struct JointTelemetry
{
  JointTelemetry (double p = 0, double v = 0, double e = 0)
  : position(p),
    velocity(v),
    effort(e) { }

  // Use compiler's copy constructor, destructor, assignment.

  double position;
  double velocity;
  double effort;
};

#endif
