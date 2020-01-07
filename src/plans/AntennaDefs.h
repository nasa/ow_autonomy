#ifndef AntennaDefs_H
#define AntennaDefs_H

// Constants used in antenna and camera plans.

// __BEGIN_LICENSE__
// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

#define D2R       0.01745 // pi/180
#define LongWait  5       // seconds
#define ShortWait 2       // seconds
#define VertFOV  10       // Easy value for testing.  Should be 15.
#define HorizFOV 10       // Easy value for testing.  Should be 21

#define PanMin   0
#define PanMax   6.2832   // pi*2  or 360 degrees
#define TiltMin -0.7854   // -pi/4 or 45 degrees upwards
#define TiltMax  0.7854   //  pi/4or 45 degrees downwards


#endif
