from typing import Callable, Tuple
from dataclasses import dataclass, field
from enum import Enum

from motorcontrol.roboclaw_types import Motor
from servocontrol.servo_types import ServoConfig

@dataclass
class SixWheelMotion:
    # Motor Objects:
    FRONT_LEFT   : Motor 
    FRONT_RIGHT  : Motor
    MIDDLE_LEFT  : Motor
    MIDDLE_RIGHT : Motor
    BACK_LEFT    : Motor
    BACK_RIGHT   : Motor

    # Maximum QPPS:
    FRONT_LEFT_QPPS   : int
    FRONT_RIGHT_QPPS  : int
    MIDDLE_LEFT_QPPS  : int
    MIDDLE_RIGHT_QPPS : int
    BACK_LEFT_QPPS    : int 
    BACK_RIGHT_QPPS   : int

    # Servos:
    FRONT_LEFT_STEER  : ServoConfig
    FRONT_RIGHT_STEER : ServoConfig
    BACK_LEFT_STEER   : ServoConfig
    BACK_RIGHT_STEER  : ServoConfig

    def getMotors(self) -> list[Motor]:
        motors = [self.FRONT_LEFT, 
                  self.FRONT_RIGHT, 
                  self.MIDDLE_LEFT,
                  self.MIDDLE_RIGHT, 
                  self.BACK_LEFT, 
                  self.BACK_RIGHT]
        return motors

    def getMotorConfigs(self) -> list[Tuple[Motor, int]]:
        motor_configs = [(self.FRONT_LEFT,   self.FRONT_LEFT_QPPS), 
                 (self.FRONT_RIGHT,  self.FRONT_RIGHT_QPPS),
                 (self.MIDDLE_LEFT,  self.MIDDLE_LEFT_QPPS),
                 (self.MIDDLE_RIGHT, self.MIDDLE_RIGHT_QPPS),
                 (self.BACK_LEFT,    self.BACK_LEFT_QPPS),
                 (self.BACK_RIGHT,   self.BACK_RIGHT_QPPS),]
        return motor_configs
    
    def loadMotors(self, motors : list[Motor]) -> None:
        self.FRONT_LEFT   = motors[0]
        self.FRONT_RIGHT  = motors[1]
        self.MIDDLE_LEFT  = motors[2]
        self.MIDDLE_RIGHT = motors[3]
        self.BACK_LEFT    = motors[4]
        self.BACK_RIGHT   = motors[5]
        return
    

