import json
import time

import numpy as np
from matplotlib.pyplot import flag

# ----- Custom class ----- #
from src.xArmManager import xArmManager
from src.SimulationManager import SimulationManager

is_Simulation = True


class RobotControlManager:
    def __init__(self, xArmConfigs) -> None:
        self.xarmManagers = {}
        for xArm in xArmConfigs:
            self.xarmManagers[xArmConfigs[xArm]['Mount']] = xArmManager(xArmConfigs[xArm]) if is_Simulation == False else SimulationManager()

    def SendDataToRobot(self, sharedMotions):
        for mount in self.xarmManagers.keys():
            self.xarmManagers[mount].SendDataToRobot(sharedMotions[mount])

    def DisConnect(self):
        for mount in self.xarmManagers.keys():
            self.xarmManagers[mount].DisConnect()