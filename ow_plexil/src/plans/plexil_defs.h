// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef OceanWorldsDefs_H
#define OceanWorldsDefs_H

// Constants used in OceanWATERS PLEXIL plans

// NOTE: Some of these are guessed/bogus values, and some intervals are very
// short for now to reduce testing time.

#define FILM_INTERVAL 6
#define SENSE_DIG_INTERVAL 6
#define DIG_TRENCH_TIMEOUT 10

#define STUB_TRUE true
#define STUB_FALSE false

#define INSTRUMENT_ID Integer

// Proposed, arbitrary small unitless value for zero velocity tolerance.
#define ZERO_VEL 0.01

#define NUM_JOINTS 9

// The following are related to antenna and camera.

#define LONG_WAIT  5       // seconds
#define SHORT_WAIT 2       // seconds
#define VERT_FOV  10       // Easy value for testing.  Should be 15.
#define HORIZ_FOV 10       // Easy value for testing.  Should be 21

// All values in degrees.  Made up constraints -- will adjust as needed.
#define PAN_MIN   0
#define PAN_MAX   359
#define TILT_MIN  -45
#define TILT_MAX  45

// Maximum number of crashes before assuming something has gone very wrong and
// attempting to offload all data
#define MAX_CRASHES 10

#endif
