#ifndef Unified_Interface_H
#define Unified_Interface_H

// Unified interface to OceanWATERS and OWLAT.
// Under construction!

// Utility commands; issue ROS_INFO, ROS_WARN, and ROS_ERROR, respectively.
Command log_info (...);
Command log_warning (...);
Command log_error (...);
Command log_debug (...);


////////////// Telemetry //////////////////

// Antenna
Real    Lookup PanRadians;
Real    Lookup PanDegrees;
Real    Lookup TiltRadians;
Real    Lookup TiltDegrees;

#endif
