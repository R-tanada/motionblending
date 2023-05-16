import json
import time

import numpy as np
from matplotlib.pyplot import flag

# ----- Custom class ----- #
from RobotArmControl.xArmManager import xArmManager


class RobotControlManager:
    def __init__(self, xArmConfigs) -> None:
        self.xarmManager = {}
        for xArm in xArmConfigs:
            self.xarmManager[xArmConfigs[xArm]['Mount']] = xArmManager(xArmConfigs[xArm])

    def SendDataToRobot(self, sharedMotions, isSafty):
        for mount in self.xarmManager.keys():
            self.xarmManager[mount].SendDataToRobot(sharedMotions[mount])




