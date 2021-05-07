# Script will stop neato lidar.
# Author: Brannon Vann brannon.vann@gmail.com
# License: MIT

# run this script directly from the neato directory: python3 -m scripts.stop_neato_lidar

import time
import sys

from src.neato_driver_python import neato_driver as robot

sys.path.append("..")

try:
    robot.init("/dev/neato", False)
    robot.TestMode(True)
    robot.SetLDSRotation(False)
    robot.TestMode(False)

    # close serial port
    serial.close()
    print("Done stopping neato lidar")
except:
    print("an error occurred while stoping neato lidar.")
