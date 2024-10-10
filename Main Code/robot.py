#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
from micropython import opt_level
from pybricks.experimental import run_parallel
opt_level(2)

class Robot:

    def __init__(self):
        self.wheel_diameter=81.6
        self.axle_track= 96
        self.ev3 = EV3Brick()
        self.left_motor = Motor(Port.A)
        self.right_motor = Motor(Port.B)
        self.C_motor = Motor(Port.C)
        self.D_motor = Motor(Port.D)
        self.gyro = GyroSensor(Port.S1)
        self.ev3.screen.set_font(Font(size=12))
        self.robot = DriveBase(self.left_motor, self.right_motor, self.wheel_diameter, self.axle_track)

        self.KP = 2.25 #Proportional gain
        self.KI = 0.00001 #Integral gain
        self.KD = 3.5 #Derivative gain
        self.Min_Power=20 #sets the minimum power the robot can drive to 14
        self.Max_Power=125 #sets the maximum power the robot can drive to 360

    def Stop(self): # makes the robot stop
        self.robot.stop()
        run_parallel(self.left_motor.brake(), self.right_motor.brake())
        self.robot.reset()

    def PID(self,speed, MM):
        CM = 10 * MM #Makes the MM CM by multiplying it by 10
        wait(25)
        self.gyro.reset_angle(0) #Resets the gyro angle
        wait(10)
        target_value = 0
        Integral = 0
        Last_error = 0
        while self.robot.distance() <= CM:
            error = target_value - self.gyro.angle() #Sets the error to the target value - the angle
            Pfix = error*KP #Multiplies Our Proportional by the Proportional Gain to have our Proportional total
            Integral =+ error
            Ifix = Integral*KI #Multiplies Our Integral by the Integral Gain to have our Integral total
            Derivative = error-Last_error
            Dfix = Derivative*KD #Multiplies Our Derivative by the Derivative Gain to have our Derivative total
            Correction = Pfix+Ifix+Dfix #Adds all the PID Totals to one Amount
            self.robot.drive(speed*3.6, 0-Correction)
            Last_error = error #Sets our Last error up
        Stop()

    def B_PID(self,speed, MM):
        CM = -10*MM #Makes the MM CM by multiplying it by 10
        wait(25)
        self.yro.reset_angle(0) #Resets the gyro angle
        wait(10)
        target_value = 0
        Integral = 0
        Last_error = 0
        while self.robot.distance() >= CM:
            error = target_value - self.gyro.angle() #Sets the error to the target value - the angle
            Pfix = error*KP #Multiplies Our Proportional by the Proportional Gain to have our Proportional total
            Integral =+ error
            Ifix = Integral*KI #Multiplies Our Integral by the Integral Gain to have our Integral total
            Derivative = error-Last_error
            Dfix = Derivative*KD #Multiplies Our Derivative by the Derivative Gain to have our Derivative total
            Correction = Pfix+Ifix+Dfix #Adds all the PID Totals to one Amount
            self.robot.drive(speed*-3.6, 0-Correction)
            Last_error = error #Sets our Last error up
        Stop()


    def turn(self,angle):
        target_value=angle #makes the target value the angle you gave
        Last_error=0 #sets last error to 0
        wait(25)
        self.gyro.reset_angle(0)
        wait(20) #waits 1/100 of a second
        error = target_value-self.gyro.angle() #sets error to the the turn
        while error != 0: #while theres still an error(a turn to do)
            error = target_value - self.gyro.angle() #Sets the error to the target value - the angle
            Pfix = error*KP #Multiplies Our Proportional by the Proportional Gain to have our Proportional total
            Derivative = error-Last_error
            Dfix = Derivative*KD #Multiplies Our Derivative by the Derivative Gain to have our Derivative total
            Correction = Pfix+Dfix #Adds all the PID Totals to one Amount
            if abs(Correction) > Max_Power: #if the absolute of correction is higher than 360
                Correction = Max_Power*target_value/abs(target_value) #sets the correction to 360 times target value/absolute of targer value
            elif abs(Correction) < Min_Power: #if the absolute of correction is lower than 14
                Correction = Min_Power*target_value/abs(target_value) #sets the correction to 14 times target value/absolute of targer value
            run_parallel (self.left_motor.run(speed=0-Correction), self.right_motor.run(speed=Correction))
            wait(10)
            Last_error=error #sets the error to the last error
        Stop()