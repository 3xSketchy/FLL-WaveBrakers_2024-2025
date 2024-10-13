#!/usr/bin/env pybricks-micropython


from robot import Robot
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import wait, StopWatch
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
from micropython import opt_level
from pybricks.experimental import run_parallel
opt_level(2)

robot = Robot()
        
def Run1():
    KP = 2.25 #Proportional gain
    KI = 0.00001 #Integral gain
    KD = 3.5 #Derivative gain

    robot.B_PID(40, 30, KP, KI, KD)
    robot.turn(30, KP, KI, KD)
    robot.B_PID(40, 15, KP, KI, KD)
    robot.turn(-30, KP, KI, KD)
    robot.B_PID(40, 20, KP, KI, KD)
    robot.turn(-60, KP, KI, KD)
    robot.B_PID(40, 12, KP, KI, KD)
    robot.PID(40, 14, KP, KI, KD)
    robot.turn(-30, KP, KI, KD)
    robot.B_PID(40,5, KP, KI, KD)
    run_parallel(robot.C_motor.run_angle(speed=180, rotation_angle=360), robot.D_motor.run_angle(speed=180, rotation_angle=360))