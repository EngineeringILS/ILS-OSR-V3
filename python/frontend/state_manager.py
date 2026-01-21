from typing import TypeVar, Generic, Protocol, runtime_checkable
from abc import ABC, abstractmethod
import sys
import numpy as np
import time

start_time = time.time()


""" Represents the base "data input" class with methods that stage changes for the data in the frontend interface. """
class DataInput():
    def __init__(self, name:str , unit:str):
        self.name = name
        self.unit = unit

    def readData(self) :
        raise NotImplementedError("Subclasses must implement read()")

    def getData(self):
        value = self.readData()
        if isinstance(value,float):
            return f"{self.name}: {value:.2f} {self.unit}"

        return f"{self.name}: {value} {self.unit}"
    
    def label(self):
        return self.name

""" Represents the derived "data input" class which handles different types of data. """
class BaterryVoltage(DataInput):
    def __init__(self,name):
        super().__init__(name, "V")
    def readData(self):
        #Add logic to read baterry voltage here

        #dummy data for now
        t = time.time() - start_time
        np.seterr(invalid='raise')
        try:
            volt =np.sqrt(np.sin(t))
            return(volt)
        except:
            return("ERROR Reading Voltage")

class RegolithCollected(DataInput):
    def __init__(self, name):
        super().__init__(name, "lbs")

    def readData(self):
        #ADD LOGIC TO READ REGOLITH
        # DUMMY DATA FOR NOW
        #

        #dummy data for now
        t = time.time() - start_time
        np.seterr(invalid='raise')
        try:
            regolith =np.sqrt(np.sin(t)*2)
            return(regolith)
        except:
            return("ERROR Reading Regolith")
        
class ESP32Data(DataInput):
    def __init__(self, name):
        super().__init__(name, "ESP32")
        self.drivingMode = "Autonomous Drive"
        self.currentState = "Idle"

        

        self.drivingModeToggle = True
        self.stateTogggle = 1

    def readData(self):
        #ADD ESP32 Logic

        return self.drivingMode , self.currentState
    def toggleDrivingMode(self):
        self.drivingModeToggle = not (self.drivingModeToggle)
        if self.drivingModeToggle:
            self.drivingMode = "Autonomous Drive"
        else:
            self.drivingMode = "TelepOp Drive"
    
    def toggleState(self):

        # Add togleing state logic

        # this is dummy data for testimg gui 
        self.stateTogggle += 1
        if self.stateTogggle > 3:
            self.stateTogggle = 1
        match self.stateTogggle:
            case 1:
                self.currentState = "Digging..."
            case 2:
                self.currentState= "Navigating..."
            case 3:
                self.currentState ="Depsositing"
    def getData(self):
        DRMode, state = self.readData()
        return f"{self.name}: Driving Mode: {DRMode} , State :{state}"
        
class IMU (DataInput):
    def __init__(self, name):
        super().__init__(name, "")
    def readData(self):
        ## Put code to get data here
        #DUMMY CODE GET RID AFTER
        t = time.time() - start_time
        np.seterr(invalid='raise')
        try:
            x =np.sqrt(np.sin(t)*2)
            y =np.sqrt(np.cos(t)*2)
            z =np.sqrt(np.tan(t)*2)

            return(x,y,z)
        except:
            return("ERROR IMU ","ERROR IMU ","ERROR IMU ")
    def getData(self):
        valueX, valueY, valueZ = self.readData()
        return f"{self.name}: {valueX} m {valueY} m {valueZ} m {self.unit}"




