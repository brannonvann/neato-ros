# Script will check Neato's battery and charging status.

# Author: Brannon Vann - brannon.vann@gmail.com
# License: MIT

# run this script directly from the neato directory: python3 -m scripts.battery_check

import time
import sys

from src.neato_driver_python import neato_driver as robot

sys.path.append("..")

try:
    robot.init("/dev/neato", False)
    charger = robot.GetCharger()
    for key, value in charger.items():
        print(key + ": " + str(value))

except:
    print("An error occurred while running script")
    raise
