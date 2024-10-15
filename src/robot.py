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

        small_font = Font(size=12)
        large_font = Font(size=14)

        self.ev3.screen.set_font(size=small_font)
        
        failedsensor = 0 #assuming all sensors/motors are OK
        while True:
            try: 
                self.gyro_port = Port.S1
                self.gyro = GyroSensor(port = self.gyro_port)
            except:
                self.gyro = None
                self.ev3.screen.print("Sensor Port 1 not found")
                failedsensor += 1
            try:
                self.left_motor = Motor(port = Port.A)
            except:
                self.left_motor = None
                self.ev3.screen.print("Motor Port A not found")
                failedsensor += 1
            try:
                self.right_motor = Motor(port = Port.B)
            except:
                self.right_motor = None
                self.ev3.screen.print("Motor Port B not found")
                failedsensor += 1
            try:
                self.C_motor = Motor(port = Port.C)
            except:
                self.C_motor = None
                self.ev3.screen.print("Motor Port C not found")
                failedsensor += 1
            try:
                self.D_motor = Motor(port = Port.D)
            except:
                self.D_motor = None
                self.ev3.screen.print("Motor Port D not found")
                failedsensor += 1
            if failedsensor == 0:
                break
            else:
                failedsensor = 0
                self.ev3.screen.print("Check the cables and replug them")
                wait(2000)
        self.robot = DriveBase(self.left_motor, self.right_motor, self.wheel_diameter, self.axle_track)

        self.Min_Power=20 #sets the minimum power the robot can drive to 14
        self.Max_Power=125 #sets the maximum power the robot can drive to 360

    def Stop(self): # makes the robot stop
        self.robot.stop()
        run_parallel(self.left_motor.hold(), self.right_motor.hold())

    def check_drive_direction(self, speed, drive_distance): #Will Make the PID a forward or a backwards

        if speed * drive_distance > 0:  # forward driving
            drive_direction = 1
        else:   # backward driving
            drive_direction = -1
        
        drive_speed = drive_direction * fabs(speed)
        return drive_speed, drive_direction

    def PID(self,speed, distance, KP, KI, KD):
        wait(25)
        self.gyro.reset_angle(0) #Resets the gyro angle

        Distance = self.robot.distance()

        target_value = 0
        Integral = 0
        Last_error = 0

        drive_speed, drive_direction = self.check_drive_direction(speed , distance)

        CM = 10 * distance #Makes the MM CM by multiplying it by 10

        while fabs(self.robot.distance() - Distance) <= fabs(CM):
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

    def Curve(self,speed, distance, angle, KP, KI, KD):
        wait(25)
        self.gyro.reset_angle(0) #Resets the gyro angle
        
        Distance = self.robot.distance()

        target_value = 0
        Integral = 0
        Last_error = 0

        drive_speed, drive_direction = self.check_drive_direction(speed , distance)

        CM = 10 * distance #Makes the MM CM by multiplying it by 10

        while fabs(self.robot.distance() - Distance) <= CM: #while The absolute of the robot distance travelled is smaller than the cm
            if angle > target_value:
                target_value += angle/abs(angle)

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

    def turn(self,angle, KP, KI, KD):
        target_value=angle #makes the target value the angle you gave
        Last_error=0 #sets last error to 0

        wait(25)
        self.gyro.reset_angle(0)

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
