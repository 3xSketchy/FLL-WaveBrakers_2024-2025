#!/usr/bin/env pybricks-micropython
from robot import Robot
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import wait, StopWatch
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
from micropython import opt_level
from pybricks.experimental import run_parallel
opt_level(3)

robot = Robot()

def Clearmap():
    KP=3
    KD=2.5
    KI=0.000001
    # Octopus Start
    robot.PID(60,30,KP,KI,KD)
    robot.turn(-35,KP)
    robot.PID(65,18,KP,KI,KD)
    robot.PID(-65,8,KP,KI,KD)
    robot.turn(-47,KP)
    robot.PID(75,20,KP,KI,KD)
    robot.PID(-55,23,KP,KI,KD)
    # Octopus End
    robot.turn(60,KP)
    robot.PID(70,40,KP,KI,KD)
    robot.turn(-65,KP)
    robot.PID(80,50,KP,KI,KD)
    robot.turn(-20,KP)
    robot.PID(60,20,KP,KI,KD)
    robot.turn(15,KP)
    robot.PID(65,15,KP,KI,KD)
    robot.turn(25,KP)
    robot.PID(60,15,KP,KI,KD)
    robot.turn(-60,KP)
    robot.PID(80,50,KP,KI,KD)
    robot.turn(-50,KP)
    robot.PID(90,60,KP,KI,KD)
def Skibidy():
    robot.PID(70, 20)
    robot.turn()