from typing import TypeVar, Generic, Protocol, runtime_checkable
from abc import ABC, abstractmethod
import sys
import numpy as np
import time
start_time = time.time()
t = time.time() - start_time

""" Represents the base "data input" class with methods that stage changes for the data in the frontend interface. """
class DataInput():
    def __init__(self, value: int):
        self.value = value

""" Represents the derived "data input" class which handles different types of data. """
class DerivedDataInput(DataInput):
    def __init__(self, value: int, extra_param: str) :
        self.value = value
        self.value2 = extra_param


class BatteryData(DataInput):
    def __init__(self, value, voltage: float):
        super().__init__(value)
        

    def read(self):
        t = time.time() - start_time
        self.voltage = np.sin(t)
        return self.voltage
        