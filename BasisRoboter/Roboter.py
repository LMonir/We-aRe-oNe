from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import operator

class Roboter():
    def __init__(self):
        self.ev3 = EV3Brick()
        self.motorB = Motor(Port.B)
        self.motorC = Motor(Port.C)
        self.licht1 = ColorSensor(Port.S1)
        self.licht2 = ColorSensor(Port.S2)
        self.driveBase = DriveBase(self.motorB, self.motorC, 94, 130)

        self.ops = {'>': operator.gt,
           '<': operator.lt,
           '>=': operator.ge,
           '<=': operator.le,
           '=': operator.eq,
           '!=': operator.ne}

    def beep(self):
        self.ev3.speaker.beep()

    def beschleunigen(self, v:int, step=40):
        v_start = self.driveBase.state()[1]
        if abs(v - v_start) < step:
            return
        delta_v = (v - v_start)/step
        for i in range(1, step+1):
            self.driveBase.drive(v_start + i * delta_v, 0)
            wait(30)

    def drive(self, v:int):
        self.beschleunigen(v)
        self.driveBase.drive(v, 0)

    def driveUntil(self, v:int, v_end:int, fn, *args):
        self.driveBase.reset()
        self.beschleunigen(v)
        self.driveBase.drive(v, 0)
        while fn(*args):
            pass
        self.beschleunigen(v_end)

    def unlockMotor(self):
        self.driveBase.stop()
    
    def brake(self):
        self.motorB.brake()
        self.motorC.brake()

    def resetMotor(self, motor:Motor, angle=0):
        motor.reset_angle(angle)

    def getMotorLambda(self, operator):
        return lambda motor, x: self.ops[operator](motor.angle(), x)

    def getSensorReflectionLambda(self, operator):
        return lambda sensor, x: self.ops[operator](sensor.reflection(), x)

