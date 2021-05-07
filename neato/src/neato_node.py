#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2021, Brannon Vann
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * The name Brannon Vann may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

## neato robot node to connect neato_driver to ros.
## publishes: /neato/battery_state, /neato/bumper_event, /neato/button_event, /neato/sensors, /odom, and /scan
## subscribes to: /cmd_vel

import rospy
import tf
import time
from std_msgs.msg import String
from math import sin, cos, atan2, pi

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan, BatteryState
from neato.msg import ButtonEvent, BumperEvent, Sensors

import neato_driver_python.neato_driver as robot

# params: general
publish_frequency = rospy.get_param("neato/publish_frequency", 10)
port = rospy.get_param("neato/port", "/dev/neato")
base_frame_id = rospy.get_param("neato/base_frame_id", "base_link")
wheel_track = rospy.get_param(
    "neato/wheel_track", 0.240
)  # distance between the centers of the two wheels

# params: cmd_vel
velocity_max_x = rospy.get_param(
    "neato/velocity_max_x", 0.3
)  # neato's absolute max is 0.3 m/s

velocity_max_z = rospy.get_param("neato/velocity_max_z", 0)  # 0 = unlimited


# params: scan
enable_scan = rospy.get_param("neato/enable_scan", True)
scan_frame_id = rospy.get_param("neato/scan_frame_id", "sensor_laser")

# params: buttons
enable_buttons = rospy.get_param("neato/enable_buttons", True)

# params: sensors
enable_sensors = rospy.get_param("neato/enable_sensors", True)

# params: bumpers
enable_bumpers = rospy.get_param("neato/enable_bumpers", True)

# params: battery
enable_battery = rospy.get_param("neato/enable_battery", True)

debug_neato_driver = rospy.get_param("neato/debug_neato_driver", False)


# globals: ros - general
current_time = None
last_time = None
delta_time = None

# globals: cmd_vel
cmd_vel = None
moving_prev = False

# globals: odom
odom_pub = None
odom_broadcaster = None
left_wheel_pos_prev = 0.0  # previous left wheel position in meters
right_wheel_pos_prev = 0.0  # previous right wheel position in meters
x = 0
y = 0
th = 0

# globals: scan
scan_pub = None
scan = None

# globals: sensors
sensors_pub = None
digital_sensors_prev = None
digital_sensors = None  # staged here for access from multiple publishers
analog_sensors_prev = None

# globals: bumpers
bumper_pub = None

# globals: buttons
button_pub = None
buttons_prev = None

# globals: battery
battery_pub = None


def main():
    global current_time, last_time, delta_time

    rospy.init_node("neato")

    rate = rospy.Rate(publish_frequency)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    init_neato()
    init_cmd_vel()
    init_odom()

    if enable_scan:
        init_scan()
    if enable_buttons:
        init_buttons()
    if enable_bumpers or enable_sensors:
        read_digital_sensors()  # call twice to initilize prev and current values
        read_digital_sensors()
    if enable_bumpers:
        init_bumpers()
    if enable_sensors:
        init_sensors()
    if enable_battery:
        init_battery()

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        delta_time = (current_time - last_time).to_sec()

        if enable_scan:
            pub_scan()
        if enable_buttons:
            pub_buttons()
        if enable_bumpers or enable_sensors:
            read_digital_sensors()
        if enable_bumpers:
            pub_bumpers()
        if enable_sensors:
            pub_sensors()
        if enable_battery:
            pub_battery()

        handle_cmd_vel()
        pub_odom()

        last_time = current_time
        rate.sleep()


def init_neato():
    robot.init(port, debug_neato_driver)
    robot.TestMode(True)
    robot.PlaySound(robot.Sounds.WakingUp)
    time.sleep(3)
    robot.SetLED(robot.BacklightStatus.On, robot.ButtonColors.Green)


def init_cmd_vel():
    # neato will listen for cmd_vel messages and
    # drive the motors according the the commands.
    rospy.Subscriber("cmd_vel", Twist, subscribe_cmd_vel)


def subscribe_cmd_vel(incoming_cmd_vel):
    global cmd_vel
    cmd_vel = incoming_cmd_vel


def handle_cmd_vel():
    global moving_prev
    if cmd_vel:
        th = cmd_vel.angular.z * (wheel_track / 2)

        # limit max vel z
        if velocity_max_z:
            th = min(th, velocity_max_z)

        dist_left = cmd_vel.linear.x - th
        dist_right = cmd_vel.linear.x + th
        req_velocity = abs(max(dist_left, dist_right))
        drive_vel = min(req_velocity, 0.3)  # .3 m/s is the max allowed by neato api

        # limit max vel x
        if velocity_max_x:
            drive_vel = min(drive_vel, velocity_max_x)

        if drive_vel == 0:
            if moving_prev:
                robot.SetMotorWheels(
                    1, 1, 1
                )  # 0,0,0 does not stop Neato. Issue 1,1,1 to go forward 1mm and stop.
        else:
            robot.SetMotorWheels(
                int(dist_left * 1000), int(dist_right * 1000), int(drive_vel * 1000)
            )

        moving_prev = drive_vel > 0


def init_odom():
    # neato will publish it's movements using the odom topcic
    global odom_pub, odom_broadcaster, left_wheel_pos_prev, right_wheel_pos_prev

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    motors = robot.GetMotors(leftWheel=True, rightWheel=True)
    left_wheel_pos_prev = motors.get("LeftWheel_PositionInMM") / 1000.0
    right_wheel_pos_prev = motors.get("RightWheel_PositionInMM") / 1000.0


def pub_odom():
    global left_wheel_pos_prev, right_wheel_pos_prev, x, y, th

    motors = robot.GetMotors(leftWheel=True, rightWheel=True)
    # {'LeftWheel_RPM': 0, 'LeftWheel_Load%': 0, 'LeftWheel_PositionInMM': 0, 'LeftWheel_Speed': 0,
    # 'RightWheel_RPM': 0, 'RightWheel_Load%': 0, 'RightWheel_PositionInMM': 0, 'RightWheel_Speed': 0}

    left_wheel_pos = motors.get("LeftWheel_PositionInMM") / 1000.0
    right_wheel_pos = motors.get("RightWheel_PositionInMM") / 1000.0

    delta_left_wheel = (
        left_wheel_pos - left_wheel_pos_prev
    )  # left wheel delta in meters
    delta_right_wheel = (
        right_wheel_pos - right_wheel_pos_prev
    )  # right wheel delta in meters

    left_wheel_pos_prev = left_wheel_pos
    right_wheel_pos_prev = right_wheel_pos

    ds = (
        delta_left_wheel + delta_right_wheel
    ) / 2.0  # robot traveled distance in meters
    dth = atan2(delta_right_wheel - delta_left_wheel, wheel_track)  # turn angle

    x += ds * cos(th + dth / 2.0)
    y += ds * sin(th + dth / 2.0)
    th += dth

    # prepare tf from base_link to odom
    quaternion = Quaternion()
    quaternion.z = sin(th / 2.0)
    quaternion.w = cos(th / 2.0)

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.0), odom_quat, current_time, base_frame_id, "odom"
    )

    # publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.0), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = base_frame_id
    odom.twist.twist.linear.x = ds / delta_time
    odom.twist.twist.angular.z = dth / delta_time

    # publish the message
    odom_pub.publish(odom)


def init_scan():
    # neato will publish it's lidar data
    global scan_pub, scan

    scan_pub = rospy.Publisher("scan", LaserScan, queue_size=10)

    scan = LaserScan(header=rospy.Header(frame_id=scan_frame_id))
    scan.angle_min = 0.0
    scan.angle_max = 359.0 * pi / 180.0
    scan.angle_increment = pi / 180.0
    scan.range_min = 0.016
    scan.range_max = 6.0

    robot.SetLDSRotation(True)


def pub_scan():
    scan.header.stamp = current_time
    scan.header.frame_id = scan_frame_id

    scan.ranges = []
    scan.intensities = []

    scan_reading = robot.GetLDSScan()

    # {0: [5009, 12, 0], ... ,359: [5060, 5, 0], 'ROTATION_SPEED': 5.07}
    # {0: [distance_code_in_mm, normalized_spot_intensity, error_code]}

    for i in range(360):
        if scan_reading[i][2] == 0:
            scan.ranges.append(scan_reading[i][0] / 1000.0)
            scan.intensities.append(scan_reading[i][1])
        else:  # error condition, ignore
            scan.ranges.append(0)
            scan.intensities.append(0)

    # publish the message
    scan_pub.publish(scan)


def read_digital_sensors():
    global digital_sensors_prev, digital_sensors

    digital_sensors_prev = digital_sensors
    digital_sensors = robot.GetDigitalSensors()

    # {'SNSR_DC_JACK_CONNECT': False, 'SNSR_DUSTBIN_IS_IN': True, 'SNSR_LEFT_WHEEL_EXTENDED': False,
    # 'SNSR_RIGHT_WHEEL_EXTENDED': False, 'LSIDEBIT': True, 'LFRONTBIT': True, 'RSIDEBIT': True,
    # 'RFRONTBIT': True}


def init_sensors():
    global sensors_pub, analog_sensors_prev
    sensors_pub = rospy.Publisher("neato/sensors", Sensors, queue_size=10)
    analog_sensors_prev = robot.GetAnalogSensors()


def pub_sensors():

    # {'WallSensorInMM': 40, 'BatteryVoltageInmV': 3792, 'LeftDropInMM': 71,
    # 'RightDropInMM': 65, 'LeftMagSensor': -758, 'RightMagSensor': -758,
    # 'UIButtonInmV': 3324, 'VacuumCurrentInmA': 0, 'ChargeVoltInmV': 23985,
    # 'BatteryTemp0InC': 28, 'BatteryTemp1InC': '26', 'CurrentInmA': '87',
    # 'SideBrushCurrentInmA': '0', 'VoltageReferenceInmV': '1225',
    # 'AccelXInmG': '24', 'AccelYInmG': 16, 'AccelZInmG': 1016}

    analog_sensors = robot.GetAnalogSensors()

    sensors = Sensors()

    # some neatos supply additional fields. this is the common set from my testing.
    analog_keys = [
        "WallSensorInMM",
        "BatteryVoltageInmV",
        "LeftDropInMM",
        "RightDropInMM",
        "LeftMagSensor",
        "RightMagSensor",
        "UIButtonInmV",
        "VacuumCurrentInmA",
        "ChargeVoltInmV",
        "BatteryTemp0InC",
        "BatteryTemp1InC",
        "CurrentInmA",
        "SideBrushCurrentInmA",
        "VoltageReferenceInmV",
        "AccelXInmG",
        "AccelYInmG",
        "AccelZInmG",
    ]

    for key in analog_keys:
        setattr(sensors, key, analog_sensors[key])

    for key, value in digital_sensors_prev.items():
        setattr(sensors, key, value)

    sensors_pub.publish(sensors)


def init_bumpers():
    global bumper_pub
    bumper_pub = rospy.Publisher("neato/bumper_event", BumperEvent, queue_size=10)


def pub_bumpers():
    global digital_sensors_prev

    # {... 'LSIDEBIT': True, 'LFRONTBIT': True, 'RSIDEBIT': True, 'RFRONTBIT': True}
    for i, key in enumerate(
        [
            "LSIDEBIT",
            "LFRONTBIT",
            "RFRONTBIT",
            "RSIDEBIT",
        ]
    ):
        if digital_sensors_prev[key] != digital_sensors[key]:
            bumper_event = BumperEvent()
            bumper_event.bumper = i
            bumper_event.engaged = digital_sensors[key]
            bumper_pub.publish(bumper_event)


def init_buttons():
    global buttons_prev, button_pub
    button_pub = rospy.Publisher("neato/button_event", ButtonEvent, queue_size=10)
    buttons_prev = robot.GetButtons()


def pub_buttons():
    global buttons_prev

    buttons = robot.GetButtons()
    # {'BTN_SOFT_KEY': False, 'BTN_SCROLL_UP': False, 'BTN_START': False, 'BTN_BACK': False, 'BTN_SCROLL_DOWN': False}

    for i, key in enumerate(
        ["BTN_START", "BTN_SOFT_KEY", "BTN_BACK", "BTN_SCROLL_DOWN", "BTN_SCROLL_UP"]
    ):
        if buttons_prev[key] != buttons[key]:
            button_event = ButtonEvent()
            button_event.button = i
            button_event.engaged = buttons[key]
            button_pub.publish(button_event)

    buttons_prev = buttons


def init_battery():
    global battery_pub
    battery_pub = rospy.Publisher("neato/battery_state", BatteryState, queue_size=10)


def pub_battery():
    # {'FuelPercent': 0, 'BatteryOverTemp': False, 'ChargingActive': False,
    # 'ChargingEnabled': False, 'ConfidentOnFuel': False, 'OnReservedFuel': True,
    # 'EmptyFuel': True, 'BatteryFailure': False, 'ExtPwrPresent': True,
    # 'ThermistorPresent[0]': True, 'ThermistorPresent[1]': True,
    # 'BattTempCAvg[0]': 27, 'BattTempCAvg[1]': 25, 'VBattV': 3.8,
    # 'VExtV': 24.12, 'Charger_mAH': 0.0}

    charger = robot.GetCharger()

    battery_state = BatteryState()

    power_supply_health = 1  # POWER_SUPPLY_HEALTH_GOOD
    if charger["BatteryOverTemp"]:
        power_supply_health = 2  # POWER_SUPPLY_HEALTH_OVERHEAT
    elif charger["EmptyFuel"]:
        power_supply_health = 3  # POWER_SUPPLY_HEALTH_DEAD
    elif charger["BatteryFailure"]:
        power_supply_health = 5  # POWER_SUPPLY_HEALTH_UNSPEC_FAILURE

    power_supply_status = 3  # POWER_SUPPLY_STATUS_NOT_CHARGING
    if charger["ChargingActive"]:
        power_supply_status = 1  # POWER_SUPPLY_STATUS_CHARGING
    elif charger["FuelPercent"] == 100:
        power_supply_status = 4  # POWER_SUPPLY_STATUS_FULL

    battery_state.percentage = charger["FuelPercent"]
    battery_state.power_supply_status = power_supply_status
    battery_state.power_supply_health = power_supply_health
    battery_state.power_supply_technology = 1  # POWER_SUPPLY_TECHNOLOGY_NIMH
    battery_state.present = charger["FuelPercent"] > 0

    if analog_sensors_prev:
        battery_state.voltage = analog_sensors_prev["BatteryVoltageInmV"] // 1000
        battery_state.temperature = analog_sensors_prev["BatteryTemp0InC"]
        battery_state.current = analog_sensors_prev["CurrentInmA"] // 1000

    battery_pub.publish(battery_state)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        robot.PlaySound(robot.Sounds.UserTerminatedCleaning)
        robot.SetLED(robot.BacklightStatus.Off, robot.ButtonColors.Off)
        robot.SetLDSRotation(False)
        robot.TestMode(False)
