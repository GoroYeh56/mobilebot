#! /usr/bin/python
import lcm
from time import sleep
import sys
sys.path.append("lcmtypes")

from lcmtypes import mbot_encoder_t
from lcmtypes import mbot_imu_t
from lcmtypes import mbot_motor_command_t
from lcmtypes import odometry_t
from lcmtypes import pose_xyt_t
from lcmtypes import robot_path_t
from lcmtypes import timestamp_t

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")

stop_command = mbot_motor_command_t()
stop_command.trans_v = 0.0
stop_command.angular_v = 0.0

drive_command = mbot_motor_command_t()
drive_command.trans_v = 0.25 #go 0.25m in 1s
drive_command.angular_v = 0.0

turn_command = mbot_motor_command_t()
turn_command.trans_v = 0.0
turn_command.angular_v = 3.1415/2.0 #turn 180 in 2s

lc.publish("MBOT_MOTOR_COMMAND",drive_command.encode())
sleep(2.0)
lc.publish("MBOT_MOTOR_COMMAND",turn_command.encode())
sleep(2.0)
lc.publish("MBOT_MOTOR_COMMAND",drive_command.encode())
sleep(2.0)
lc.publish("MBOT_MOTOR_COMMAND",turn_command.encode())
sleep(2.0)
lc.publish("MBOT_MOTOR_COMMAND",drive_command.encode())
sleep(2.0)
lc.publish("MBOT_MOTOR_COMMAND",turn_command.encode())
sleep(2.0)
lc.publish("MBOT_MOTOR_COMMAND",drive_command.encode())
sleep(2.0)
lc.publish("MBOT_MOTOR_COMMAND",turn_command.encode())
sleep(2.0)
lc.publish("MBOT_MOTOR_COMMAND",stop_command.encode())
