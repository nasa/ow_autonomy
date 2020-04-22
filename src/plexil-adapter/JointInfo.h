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


#endif
