#ifndef AntennaDefs_H
#define AntennaDefs_H

// Constants used in antenna and camera plans.

// __BEGIN_LICENSE__
// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

#define LongWait  5       // seconds
#define ShortWait 2       // seconds
#define VertFOV  10       // Easy value for testing.  Should be 15.
#define HorizFOV 10       // Easy value for testing.  Should be 21

// All values in degrees.  Made up constraints -- will adjust as needed.
#define PanMin   0
#define PanMax   359 
#define TiltMin  -45
#define TiltMax  45


#endif
