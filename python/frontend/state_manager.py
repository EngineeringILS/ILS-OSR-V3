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
        value = str(self.readData())
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
        self.state = "Autonomous Drive"

        self.tog = True
    def readData(self):
        #ADD ESP32 Logic
        return self.state
    def toggleState(self):
        self.tog = not (self.tog)
        if self.tog:
            self.state = "Autonomous Drive"
        else:
            self.state = "TelepOp Drive"





