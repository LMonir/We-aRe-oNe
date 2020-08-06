#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from Roboter import Roboter


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
roboter = Roboter()

# Write your program here.

waitUntilMotorAngle = roboter.getMotorLambda("<")
roboter.resetMotor(roboter.motorB)
roboter.driveUntil(800, 100, waitUntilMotorAngle, roboter.motorB, 3*360)
roboter.unlockMotor()
#roboter.resetMotor(roboter.motorB)
#roboter.driveUntil(100, 0, waitUntilMotorAngle, roboter.motorB, 3*360)
#roboter.unlockMotor()
roboter.brake()

