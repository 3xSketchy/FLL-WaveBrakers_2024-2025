#!/usr/bin/env pybricks-micropython


from robot import Robot
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import wait, StopWatch
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
from micropython import opt_level
from pybricks.experimental import run_parallel
opt_level(2)

robot = Robot()

robot.B_PID(40, 30)
robotturn(30)
robot.B_PID(40, 15)
robot.turn(-30)
robot.B_PID(40, 20)
robot.turn(-60)
B_PID(40, 12)
robot.PID(40, 14)
robot.turn(-30)
robot.B_PID(40,5)
run_parallel(robot.C_motor.run_angle(speed=180, rotation_angle=360), robot.D_motor.run_angle(speed=180, rotation_angle=360))