#ifndef AntennaDefs_H
#define AntennaDefs_H

// Constants used in antenna and camera plans.

// __BEGIN_LICENSE__
// Copyright (c) 2018-2020, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

#define LONG_WAIT  5       // seconds
#define SHORT_WAIT 2       // seconds
#define VERT_FOV  10       // Easy value for testing.  Should be 15.
#define HORIZ_FOV 10       // Easy value for testing.  Should be 21

// All values in degrees.  Made up constraints -- will adjust as needed.
#define PAN_MIN   0
#define PAN_MAX   359 
#define TILT_MIN  -45
#define TILT_MAX  45


#endif
