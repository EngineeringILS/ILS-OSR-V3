"""
roboclaw_advanced_control.py
Advanced Methods for Setting up and Managing the Roboclaw MCU
"""
import time
# Roboclaw Base Library
from basicmicro import Basicmicro as Roboclaw

# FSM State Curves and Control:
# Think ECEN-248 + ECEN-214 Type Logic:
from roboclaw_control_curves import normalized_decay_array, normalized_logistic_array
from roboclaw_types import MotorCurveLUT, populate_curve_lut, MovementState, MovementSubState, SpeedConfig, MotorCurveLUTConfig, MotorControllerState, reverse_index_lut
# Fixing Function Input Types:
import typing

# For Active Input:
import sys
import tty
import termios

class RoboclawControlLoop:
    """
    State-machine driven controller for the Roboclaw MCU with strict safety boundaries.
    """
    def __init__(self, serial_port: str, baud_rate: int, controller_address: int, speed_config: SpeedConfig, debug: bool, coast_timeout: int = 10):
        # Static Environmental Parameters:
        self.serial_port: str = serial_port
        self.baud_rate: int = baud_rate
        self.controller_address: int = controller_address
        self.debug: bool = debug
        self.coast_timeout: int = coast_timeout
        # Populate LUTs
        self.curveLutConfig: MotorCurveLUTConfig = MotorCurveLUTConfig(
            # 5s @ 0.1 smooth ramp
            forward_accelerate_lut=MotorCurveLUT(speed_config.min_forward, speed_config.max_forward, 100, 0.1),
            # 2s @ 0.15 smooth decay
            forward_decelerate_lut=MotorCurveLUT(speed_config.min_forward, speed_config.max_forward, 40, 0.15),
            # 2.5s @ 0.1 smooth ramp (half the speed to cover)
            reverse_accelerate_lut=MotorCurveLUT(speed_config.min_reverse, speed_config.max_reverse, 50, 0.1),
            # 1s @ 0.15 smooth decay 
            reverse_decelerate_lut=MotorCurveLUT(speed_config.min_reverse, speed_config.max_reverse, 20, 0.15),
            # This is just flat, at whatever the turning speed is.
            turning_lut=MotorCurveLUT(speed_config.turn, speed_config.turn, 10, 1),
            # This decay aggressively takes whatever the current speed is, and damps it to zero in 1 second at a 0.5 decay.
            general_braking_lut=MotorCurveLUT(speed_config.brake, speed_config.max_forward, 20, 0.5),
            # This is just flat, at 0
            stopped_lut=MotorCurveLUT(0,0,10,1)
        )
        # Populate the LUTs.
        populate_curve_lut(normalized_logistic_array, self.curveLutConfig.forward_accelerate_lut)
        populate_curve_lut(normalized_decay_array, self.curveLutConfig.forward_decelerate_lut)
        populate_curve_lut(normalized_logistic_array, self.curveLutConfig.reverse_accelerate_lut)
        populate_curve_lut(normalized_decay_array, self.curveLutConfig.reverse_decelerate_lut)
        populate_curve_lut(normalized_logistic_array, self.curveLutConfig.turning_lut)
        populate_curve_lut(normalized_decay_array, self.curveLutConfig.general_braking_lut)
        populate_curve_lut(normalized_logistic_array, self.curveLutConfig.stopped_lut)

        # Dynamic Roboclaw MCU Object, State Machine Driver:
        self.is_connected = False
        self.roboclaw = Roboclaw(self.serial_port, self.baud_rate)
        self.State: MotorControllerState = MotorControllerState()

    def connect(self) -> bool:
        """Opens the Roboclaw Serial Connection and verifies firmware."""
        print(f"Attempting to Connect to Roboclaw at {self.serial_port} (BAUD: {self.baud_rate}).")
        try:
            self.roboclaw.Open()
            firmware_version = self.roboclaw.ReadVersion(self.controller_address)
            if firmware_version[0]:
                print(f"Connected to Roboclaw. Firmware: {firmware_version[1]}.")
                self.is_connected = True
                # STOP immediately if for some reason we connect and the default state on the Roboclaw is moving. 
                self.roboclaw.SpeedM1M2(address=self.controller_address, 
                                        speed1=self.curveLutConfig.stopped_lut.array[0], 
                                        speed2=self.curveLutConfig.stopped_lut.array[0])
                self.is_connected = True
                return True
            else:
                print("Failed to connect: Firmware read failed.")
                return False
        except Exception as error:
            print(f"Error Connecting to Roboclaw: {str(error)}")
            return False
        
    def ESTOP(self) -> None:
        """
        EMERGENCY STOP (ESTOP) Enable
        """
        self.roboclaw.SpeedM1M2(address=self.controller_address, m1=0, m2=0)
        # Full reconstruct - call to reset all state information, ESTOP implies the previous state was completely unsafe.
        self.State = MotorControllerState()

    def motion_state_handler(self, input_char: str) -> None:    
        ## 00. INPUT CASTING:
        # Lowercase cast check (None --> None though?)
        try:
            # Lowercase all alphabet inputs:
            input_char = input_char.lower()
        except AttributeError:
            # Leave the object be if not alphabetic:
            pass
        if input_char not in (None, 'w','a','s','d',' '):
            input_char = None  # Cast to None, cascade of garbage inputs determinsitically yields braking 
        if input_char != None and input_char != self.State.last_input:
            substate_change = True
        
        ## 01. STOPPED STATE:
        # Easy Case: Robot STOPPED --> ROBOT MOVING (or stay STOPPED/BRAKE)
        # ALL STATE CHANGES FROM THE STOPPED (TRIVIAL STATE)
        if (self.State.state == MovementState.STOPPED):
            if (input_char == None):
                self.State.state = MovementState.STOPPED
            elif (input_char == ' '):
                self.State.state = MovementState.STOPPED
            elif (input_char == 'w'):
                self.State.state = MovementState.FORWARD_DRIVE
            elif (input_char == 's'):
                self.State.state = MovementState.REVERSE_DRIVE
            elif ((input_char == 'a') or (input_char == 'd')):
                self.State.state = MovementState.TURN_DRIVE
            else:
                return
            # Assign the last inputted char, the above if/elses filter any garbage chars into a null valued return:
            self.State.last_input = input_char
            return
        
        # 02. ABSOLUTE BRAKE:
        ## ABSOLUTE BRAKING (ALWAYS GOES TO STOP, DOES NOT CARE):
        if (self.State.state == MovementState.BRAKE):
            self.state_absolute_brake()
        
        ## ALL STATE CHANGES FOR FORWARD_DRIVE
        elif(self.State.state == MovementState.FORWARD_DRIVE):
            ## FSM Space or Timeout to Brake:
            ## BRAKE/TIMEOUT CASE:
            if(input_char == None):
                if self.State.last_input != None:
                    self.State.last_input = None
                    self.State.num_inputs = 1
                    self.State.substate = MovementSubState.CONSTANT_VELOCITY ## COASTING, CONFIGURABLE TIMEOUT, BRAKE KEY AUTOMATICALLY CANCELS
                # Obey FSM, coast timeout --> brake ALWAYS
                if (self.State.last_input == None) and (self.State.num_inputs == self.coast_timeout):
                    self.State.state = MovementState.BRAKE
                    self.State.substate = MovementSubState.BRAKING
                    self.State.num_inputs = 0

                elif self.State.last_input == None:
                    self.State.num_inputs += 1

            elif(input_char == 'w'):
                if self.State.last_input != 'w':
                    # Complete Later:
                    pass
    

    ## If I'm in BRAKE, then I need to either figure out if I'm starting to brake, actively in the process of braking, or at the end of the braking --> STOPPED
    def state_absolute_brake(self):
        if (self.State.num_inputs == 0):
            next_lut_index = reverse_index_lut(self.curveLutConfig.general_braking_lut, self.State.straight_current_speed)
            if (next_lut_index < 0):
                 # No Index Found - Serious Error implying an empty array or some other unrecoverable and unsafe condition:
                self.ESTOP()
            self.State.lut_index = next_lut_index
        elif (self.State.lut_index < len(self.curveLutConfig.general_braking_lut.array) -1):
            self.State.lut_index +=1 
        else:
            self.State.state = MovementState.STOPPED
        

    def substate_change_handler(self, nextSubState: MovementSubState):
        if (self.State.state == MovementState.FORWARD_DRIVE):
            if ((self.State.substate == MovementSubState.ACCELERATING or self.State.substate == MovementSubState.CONSTANT_VELOCITY)   and nextSubState == MovementSubState.DECELERATING):
                next_lut_index = reverse_index_lut(self.curveLutConfig.forward_decelerate_lut, self.State.straight_current_speed)
                if (next_lut_index < 0):
                    # No Index Found - Serious Error implying an empty array or some other unrecoverable and unsafe condition:
                    self.ESTOP()
                else:
                    self.State.lut_index = next_lut_index
                    self.State.substate = nextSubState
            elif ((self.State.substate == MovementSubState.DECELERATING or self.State.substate == MovementSubState.CONSTANT_VELOCITY)  and nextSubState == MovementSubState.ACCELERATING):
                next_lut_index = reverse_index_lut(self.curveLutConfig.forward_accelerate_lut, self.State.straight_current_speed)
                if (next_lut_index < 0):
                    # No Index Found - Serious Error implying an empty array or some other unrecoverable and unsafe condition:
                    self.ESTOP()
                else:
                    self.State.lut_index = next_lut_index
                    self.State.substate = nextSubState
            else:
                return None # The substate did not change

        elif (self.State.state == MovementState.REVERSE_DRIVE):


    def move(self):
        """
        Configured Set Motion Output.
        """
        # TODO: Migrate from current_speed to desired_speed if decoupling logic possible with this MCU:
        # TODO: Upstream safety (abandon if C++ kept for Phase 2)
        # Set Motion Error - but if this occurs, OBC has totally lost control of the motor harness --> Ideal solution MCU relay connected to the onboard Microcontroller.
        self.State.movement_err = self.roboclaw.SpeedM1M2(address=self.controller_address, m1=self.State.m1_current_speed, m2=self.State.m2_current_speed)
    
