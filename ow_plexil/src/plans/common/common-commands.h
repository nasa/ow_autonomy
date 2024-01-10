// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef common_commands_H
#define common_commands_H

// PLEXIL command declarations for all lander ROS actions common to
// OceanWATERS and OWLAT.

// Note that in PLEXIL, commands are inherently asynchronous.  Most
// plans should use the convenient library actions (defined in
// common-interface.h) that wrap these commands in SynchronousCommand.
// However, custom behavior can be obtained by calling these commands
// directly.

Command arm_find_surface (Integer frame,
                          Boolean relative,
                          Real position[3],
                          Real normal[3],
                          Real distance,
                          Real overdrive,
                          Real force_threshold,
                          Real torque_threshold);
Command arm_unstow();
Command arm_stow();
Command arm_stop();
Command arm_move_joint (Boolean relative, Integer joint, Real angle);

// Takes an Euler angle for orientation.
Command arm_move_cartesian (Integer frame,
                            Boolean relative,
                            Real position[3],
                            Real orientation[3]);

// Takes a quaternion for orientation.
Command arm_move_cartesian_q (Integer frame,
                              Boolean relative,
                              Real position[3],
                              Real orientation[4]);

// Takes an Euler angle for orientation.
Command arm_move_cartesian_guarded (Integer frame,
                                    Boolean relative,
                                    Real position[3],
                                    Real orientation[3],
                                    Real ForceThreshold,
                                    Real TorqueThreshold);

Command camera_capture ();

Command fault_clear (Integer fault);

// Takes a quaternion for orientation.
Command arm_move_cartesian_guarded_q (Integer frame,
                                      Boolean relative,
                                      Real position[3],
                                      Real orientation[4],
                                      Real ForceThreshold,
                                      Real TorqueThreshold);

Command pan_tilt_move_joints (Real pan_degrees, Real tilt_degrees);

Command task_deliver_sample ();

Command task_discard_sample (Integer frame,
                             Boolean relative,
                             Real point[3],
                             Real height);

#endif
