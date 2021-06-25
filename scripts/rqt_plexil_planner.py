#!/usr/bin/env python

import sys

from rqt_plexil_planner.rqt_planner import MyPlugin
from rqt_gui.main import Main

plugin = 'rqt_plexil_planner'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
