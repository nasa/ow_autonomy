#ifndef Antenna_Manager_H
#define Antenna_Manager_H

enum class AntennaState;

class AntennaManager
{
 public:
  AntennaManager ();
  // using compiler's copy constructor, assignment, destructor

 private:
  AntennaState m_state;
  double m_currentPan;
  double m_goalPan;
  double m_currentTilt;
  double m_goalTilt;
};

#endif
