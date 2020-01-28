#ifndef OceanWorldsDefs_H
#define OceanWorldsDefs_H

// Constants used in OceanWorld PLEXIL plans and OwSimProxy.

// __BEGIN_LICENSE__
// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

// NOTE: Guessed/bogus values, some intervals very short for now to reduce
// testing time.

// QUESTION: which values should be constants, and which should be dynamically
// looked up from Sim?

#define FILM_INTERVAL 3
#define SENSE_DIG_INTERVAL 6
#define DIG_TRENCH_TIMEOUT 10

#define STUB_TRUE true
#define STUB_FALSE false

#define INSTRUMENT_ID Integer

// Proposed, arbitrary small unitless value for zero velocity tolerance.

#define ZERO_VEL 0.01

#endif
