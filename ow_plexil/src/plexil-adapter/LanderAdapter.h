// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Lander_Adapter
#define Lander_Adapter

// PLEXIL adapter base class for OceanWATERS and JPL's OWLAT.

#include "PlexilAdapter.h"

// PLEXIL
#include <AdapterExecInterface.hh>

class LanderInterface;

class LanderAdapter : public PlexilAdapter
{
 public:
  static bool checkAngle (const char* name, double val, double min, double max,
                          double tolerance);
  static LanderInterface* s_interface;
  static float PanMinDegrees;
  static float PanMaxDegrees;
  static float TiltMinDegrees;
  static float TiltMaxDegrees;
  static float PanTiltInputTolerance;

  LanderAdapter (PLEXIL::AdapterExecInterface&, PLEXIL::AdapterConf*);
  LanderAdapter () = delete;
  virtual ~LanderAdapter () = default;
  LanderAdapter (const LanderAdapter&) = delete;
  LanderAdapter& operator= (const LanderAdapter&) = delete;

  virtual void lookupNow (const PLEXIL::State&, PLEXIL::LookupReceiver&) { }

 protected:
  bool initialize (LanderInterface*);
};

extern "C" {
  void initLanderAdapter();
}

#endif
