#!/usr/bin/env python

import sys

from rqt_pincher_gui.my_module import MyPlugin
from rqt_gui.main import Main

plugin = 'rqt__pincher_gui'
main = Main(filename=plugin)
sys.exit(main.main(standalone=plugin))
