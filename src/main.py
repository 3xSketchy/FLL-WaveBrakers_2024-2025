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
    KP=2.5
    KI=1.00001
    KD=3.5
    robot.PID(60,50,KP,KI,KD)
    robot.turn(30,KP,KD)
    robot.PID(60,19,KP,KI,KD)
    robot.turn(-110,KP,KD)
    robot.D_motor.run_angle(speed=180, rotation_angle=160)
    wait(100)
    robot.PID(40,27,KP,KI,KD)
    wait(100)
    robot.D_motor.run_angle(speed=180, rotation_angle=-130)
    wait(100)
    robot.PID(-60,25,KP,KI,KD)
def Clearmap():
    KP=2
    KD=2.5
    KI=0.000001
    robot.PID(55, 30, KP,KI , KD)
    robot.turn(-45,KP,KD)
    wait(200)
    robot.PID(55,18,KP,KI,KD)
    robot.PID(-55,8,KP,KI,KD)
    robot.turn(-30,KP,KD)
    robot.PID(55,15,KP,KI,KD)
    robot.PID(-55,20,KP,KI,KD)
    robot.turn(60,KP,KD)
    robot.PID(70,40,KP,KI,KD)
    wait(100)
    robot.turn(-65,KP,KD)
    robot.PID(80,50,KP,KI,KD)
    robot.turn(-10,KP,KD)
    robot.PID(50,20,KP,KI,KD)
    robot.PID(-50,10,KP,KI,KD)
    robot.turn(15,KP,KD)
    robot.PID(55,25,KP,KI,KD)
robot.gyro_calib()
Clearmap()
