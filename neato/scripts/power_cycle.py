# Script will power cycle neato.

# Author: Brannon Vann - brannon.vann@gmail.com
# License: MIT

# run this script directly from the neato directory: python3 -m scripts.power_cycle

import time
import sys

from src.neato_driver_python import neato_driver as robot

sys.path.append("..")

try:
    robot.init("/dev/neato", False)
    robot.SetSystemMode(robot.SystemModes.PowerCycle)
except:
    print("An error occurred while running script")
