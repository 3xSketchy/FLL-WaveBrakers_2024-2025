#!/usr/bin/env pybricks-micropython
from robot import Robot
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import wait, StopWatch
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
from micropython import opt_level
from pybricks.experimental import run_parallel
opt_level(3)

robot = Robot()

def Run():

    robot.PID(40, -30)
    robot.turn(30)
    robot.PID(40, -15)
    robot.turn(-30)
    robot.PID(40, -20)
    robot.turn(-60)
    robot.PID(40, -12)
    robot.PID(40, 14)
    robot.turn(-30)
    robot.PID(40)
    def C_motor1():
        robot.C_motor.run_angle(speed=180, rotation_angle=360)
    def D_motor1():
        robot.D_motor.run_angle(speed=180, rotation_angle=360)
    run_parallel(C_motor1, D_motor1)
def Run1():
    KP=4
    KI=1.00001
    KD=3.5
    robot.PID(70,50,KP,KI,KD)
    robot.turn(30,KP,KD)
    robot.PID(70,25,KP,KI,KD)
    robot.turn(-120,KP,KD)
    robot.D_motor.run_angle(speed=180, rotation_angle=40)
robot.gyro_calib()
Run1()