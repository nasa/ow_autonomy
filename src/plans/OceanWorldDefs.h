#ifndef OceanWorldsDefs_H
#define OceanWorldsDefs_H

// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.

// Constants used in OceanWATERS PLEXIL plans and OwSimProxy.

// NOTE: Some of these are guessed/bogus values, and some intervals are very
// short for now to reduce testing time.

#define FILM_INTERVAL 3
#define SENSE_DIG_INTERVAL 6
#define DIG_TRENCH_TIMEOUT 10

#define STUB_TRUE true
#define STUB_FALSE false

#define INSTRUMENT_ID Integer

// Proposed, arbitrary small unitless value for zero velocity tolerance.
#define ZERO_VEL 0.01

#define NUM_JOINTS 8

#endif
