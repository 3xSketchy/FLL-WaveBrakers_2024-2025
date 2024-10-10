#!/usr/bin/env pybricks-micropython


from robot import Robot
from pybricks.ev3devices import (Motor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
from micropython import opt_level
from pybricks.experimental import run_parallel
opt_level(2)

robot = Robot()

B_PID(40, 30)
turn(30)
B_PID(40, 15)
turn(-30)
B_PID(40, 20)
turn(-60)
B_PID(40, 12)
PID(40, 14)
turn(-30)
B_PID(40,5)
run_parallel(C_motor.run_angle(speed=180, rotation_angle=360), D_motor.run_angle(speed=180, rotation_angle=360))