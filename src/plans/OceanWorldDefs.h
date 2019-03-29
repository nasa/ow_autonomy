#ifndef OceanWorldsDefs_H
#define OceanWorldsDefs_H

// Constants used in OceanWorld PLEXIL plans and OwSimProxy.

// NOTE: Guessed/bogus values, some intervals very short for now to reduce
// testing time.

// QUESTION: which values should be constants, and which should be dynamically
// looked up from Sim?

#define DigDataGenerationRate    10.0
#define DigEnergyConsumptionRate 50.0
#define FilmInterval 3
#define SenseDigInterval 6
#define DigTrenchTimeout 10

#define StubTrue true
#define StubFalse false

#define InstrumentId Integer

#endif
