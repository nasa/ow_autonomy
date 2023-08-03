// The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
// Research and Simulation can be found in README.md in the root directory of
// this repository.

#ifndef Lander_Adapter
#define Lander_Adapter

// PLEXIL adapter base class for OceanWATERS and JPL's OWLAT.  In
// principle, other simulators' PlexilAdapter classes could be derived
// from this one.

#include "PlexilAdapter.h"

// PLEXIL
#include <AdapterExecInterface.hh>
#include <LookupReceiver.hh>

class LanderInterface;

class LanderAdapter : public PlexilAdapter
{
 public:
  static bool checkAngle (const char* name, double val, double min, double max,
                          double tolerance);

  // Pointer to concrete instance (OwAdapter or OwlatAdapter)
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
  virtual bool initialize (PLEXIL::AdapterConfiguration*) override;

 protected:
  template<typename T>
  auto lookupHandler (T val)
  {
    return [&] (const PLEXIL::State&, PLEXIL::LookupReceiver* r) {
             r->update(PLEXIL::Value(val));
           };
  }
};

#endif
