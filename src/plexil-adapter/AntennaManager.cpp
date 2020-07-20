#include "AntennaManager.h"

enum class AntennaState
  { resting,            // fully at rest, nothing commanded
    pre_pan,            // pan joint at rest, pan commanded
    panning,            // pan in progress
    pre_tilt,           // tilt joint at rest, tilt commanded
    tilting,            // tilt in progress
    panning_and_tilting // pan and tilt in progress
  };  

AntennaManager::AntennaManager ()
  : m_state (AntennaState::resting),
    m_currentPan (0),
    m_goalPan (0),
    m_currentTilt (0),
    m_goalTilt (0)
{
}
