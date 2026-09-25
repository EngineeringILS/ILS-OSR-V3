"""
ILS OSR V3 2026 Data Collection Types
"""

from typing import Callable, Tuple
from dataclasses import dataclass, field
from enum import Enum

@dataclass
class imuData:
    imuTime  : float = 0.0
    time     : float = 0.0
    ax : float = 0.0
    ay : float = 0.0
    az : float = 0.0
    gx : float = 0.0
    gy : float = 0.0
    gz : float = 0.0
    mx : float = 0.0
    my : float = 0.0
    mz : float = 0.0

    def makeHeader(self) -> str:
        """ Generates a CSV Header for imuData CSV Writing"""
        headerstring : str = "t_imu,t_self,ax,ay,az,gx,gy,gz,mx,my,mz"
        return headerstring

    def makeLine(self) -> str:
        """Transforms imuData into a CSV Writable String"""
        linestring : str = f"{self.imuTime},{self.time},{self.ax},{self.ay},{self.az},{self.gx},{self.gy},{self.gz},{self.mx},{self.my},{self.mz}"
        return linestring