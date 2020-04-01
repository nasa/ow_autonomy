/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */

// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.

#ifndef OW_AUTONOMY_JOINTINFO_H
#define OW_AUTONOMY_JOINTINFO_H

// Support for lander joint information, based on /joint_states message.

#include <string>
#include <vector>
#include <map>

// Enumerate OW's JointState 'name' array, as integer indices into this and its
// other arrays.  Can't use enum classes very well for this, so going old-school.

enum JointName { j_ant_pan = 0, j_ant_tilt,
                 j_dist_pitch, j_hand_yaw,
                 j_prox_pitch, j_scoop_yaw,
                 j_shou_pitch, j_shou_yaw
};

struct JointInfo
{
  JointInfo (double p = 0, double v = 0, double e = 0)
  : position(p),
    velocity(v),
    effort(e) { }

  // Use compiler's copy constructor, destructor, assignment

  double position;
  double velocity;
  double effort;
};

typedef std::map<int, JointInfo> JointMap;

const std::vector<std::string> JointNames {
  "Pan", "Tilt", "DistalPitch", "HandYaw", "ProximalPitch",
  "ScoopYaw", "ShoulderPitch", "ShoulderYaw"
};



#endif
