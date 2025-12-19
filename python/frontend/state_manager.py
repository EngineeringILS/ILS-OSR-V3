from typing import TypeVar, Generic, Protocol, runtime_checkable
from abc import ABC, abstractmethod
import sys

""" Represents the base "data input" class with methods that stage changes for the data in the frontend interface. """
class DataInput():
    def __init__(self, value: int):
        self.value = value

""" Represents the derived "data input" class which handles different types of data. """
class DerivedDataInput(DataInput):
    def __init__(self, value: int, extra_param: str) :
        self.value = value
        self.value2 = extra_param