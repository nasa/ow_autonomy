#ifndef Ow_Executive_H
#define Ow_Executive_H

// Autonomy Executive for OceanWATERS
// Embeds a PLEXIL executive and wraps a PLEXIL application.

// __BEGIN_LICENSE__
// Copyright (c) 2018-2019, United States Government as represented by the
// Administrator of the National Aeronautics and Space Administration. All
// rights reserved.
// __END_LICENSE__

#include <string>

class OwExecutive
{
 public:
  static OwExecutive* instance();
  // Using compiler's default constructor and destructor

  bool initialize ();
  bool runPlan (const std::string& filename);

 private:
  OwExecutive (const OwExecutive&);            // undefined, no copying
  OwExecuitve& operator= (const OwExecutive&); // undefined, no assignment
};

#endif
