#ifndef Ow_Executive_H
#define Ow_Executive_H

// Autonomy Executive for OceanWATERS.

// The implementation embeds a PLEXIL executive and application.  Because only
// one PLEXIL executive can run in one process, this class is a singleton.

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
  OwExecutive() { }
  ~OwExecutive();

  bool initialize ();
  bool runPlan (const std::string& filename);

 private:
  OwExecutive (const OwExecutive&);            // undefined (singleton)
  OwExecutive& operator= (const OwExecutive&); // undefined (singleton)
  static OwExecutive* m_instance;
};

#endif
