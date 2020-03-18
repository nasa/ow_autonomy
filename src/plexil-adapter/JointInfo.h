#ifndef OW_AUTONOMY_JOINTINFO_H
#define OW_AUTONOMY_JOINTINFO_H

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


#endif
