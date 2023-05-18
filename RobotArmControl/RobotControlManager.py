import json
import time

import numpy as np
from matplotlib.pyplot import flag

# ----- Custom class ----- #
from RobotArmControl.xArmManager import xArmManager


class RobotControlManager:
    def __init__(self, xArmConfigs) -> None:
        self.xarmManagers = {}
        for xArm in xArmConfigs:
            self.xarmManagers[xArmConfigs[xArm]['Mount']] = xArmManager(xArmConfigs[xArm])

    def SendDataToRobot(self, sharedMotions):
        for mount in self.xarmManagers.keys():
            self.xarmManagers[mount].SendDataToRobot(sharedMotions[mount])

    def DisConnect(self):
        for mount in self.xarmManagers.keys():
            self.xarmManagers[mount].DisConnect()
