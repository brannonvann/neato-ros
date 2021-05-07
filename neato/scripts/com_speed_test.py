# Script will test the communication speed and computer processing
# speed of the host computer and neato combined.

# Author: Brannon Vann brannon.vann@gmail.com
# License: MIT

# run this script directly from the neato directory: python3 -m scripts.com_speed_test


import time
import sys

from src.neato_driver_python import neato_driver as robot

sys.path.append("..")

print("Testing Neato and Computer Performance")
print("Serial read times depend on the neato and computer hosting the connection.")

robot.init("/dev/neato", False)
robot.TestMode(True)
start = time.time()
i = 0
times = 10

fns = [
    robot.GetLDSScan,
    robot.GetButtons,
    robot.GetCharger,
    robot.GetDigitalSensors,
    robot.GetAnalogSensors,
]

# single function tests

for fn in fns:
    start = time.time()
    fn()
    end = time.time()
    print(fn.__name__ + " " + str(end - start) + "s")

# time for 10 cycles, all functions
while i < times:
    for fn in fns:
        fn()
    i += 1
end = time.time()

print(
    str(end - start)
    + "s to execute "
    + ", ".join(map(lambda fn: fn.__name__, fns))
    + " "
    + str(times)
    + " times"
)


# functions per second
counter = 0
start = time.time()
while time.time() - start < 1:
    for fn in fns:
        fn()
    counter += 1

print(
    str(counter)
    + " executions per second - functions: "
    + ", ".join(map(lambda fn: fn.__name__, fns))
)
