#!/usr/bin/env python
#The Notices and Disclaimers for Ocean Worlds Autonomy Testbed for Exploration
#Research and Simulation can be found in README.md in the root directory of
#this repository.

import sys

from rqt_plexil_plan_selection.rqt_plexil_plan_selection import PlexilPlanSelectionGUI
from rqt_gui.main import Main

plugin = 'rqt_plexil_plan_selection'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
