#ifndef OW_AUTONOMY_SUPPORT_H
#define OW_AUTONOMY_SUPPORT_H

// Misc support code

// The 'name' array of the JointStates message; use as index into other arrays.
// TODO: convert to C++11 enum class.

enum JointStateNames { j_ant_pan, j_ant_tilt,
                       j_dist_pitch, j_hand_yaw,
                       j_prox_pitch, j_scoop_yaw,
                       j_shou_pitch, j_shou_yaw
};

#endif
