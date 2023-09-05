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
#include <LookupReceiver.hh>
#include <State.hh>

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
  virtual ~LanderAdapter () = 0;
  LanderAdapter (const LanderAdapter&) = delete;
  LanderAdapter& operator= (const LanderAdapter&) = delete;
  virtual bool initialize (PLEXIL::AdapterConfiguration*) override;

 protected:
  template<typename T>
  auto lookupHandler_constant (const T& val)
  {
    return [=] (const PLEXIL::State&, PLEXIL::LookupReceiver* r) {
             r->update(PLEXIL::Value(val));
           };
  }

  template<typename T, typename Class, typename Base>
  auto lookupHandler_function0 (const Class& instance,
                                T(Base::*method)() const)
  {
    return [&instance, method] (const PLEXIL::State&,
                                PLEXIL::LookupReceiver* r) {
             r->update(PLEXIL::Value((instance.*method)()));
           };
  }

  template<typename T, typename Class, typename Base>
  auto lookupHandler_function1 (const Class& instance,
                                T(Base::*method)(const std::string&) const)
  {
    return [&instance, method] (const PLEXIL::State& s,
                                PLEXIL::LookupReceiver* r) {
             std::string arg;
             s.parameters()[0].getValue(arg);
             r->update(PLEXIL::Value((instance.*method)(arg)));
           };
  }

  /* For future reference: an attempt using std::bind() that doesn't build:
  template <typename T, typename Callable>
  auto lookupHandler_callable (Callable c)
  {
    return [&] (const PLEXIL::State&, PLEXIL::LookupReceiver* r) {
             r->update(PLEXIL::Value(c()));
           };
  }
  */
};

#endif
