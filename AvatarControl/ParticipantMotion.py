# -----------------------------------------------------------------------
# Author:   Takayoshi Hagiwara (KMD)
# Created:  2021/10/6
# Summary:  操作者の動きマネージャー
# -----------------------------------------------------------------------

import threading
import time
import numpy as np
import csv

# ----- Custom class ----- #
from OptiTrack.OptiTrackStreamingManager import OptiTrackStreamingManager
from AvatarControl.GripperSensor import GripperSensorManager

# ----- Numeric range remapping ----- #
# targetMin   = 200
targetMin   = 150
targetMax   = 850
originalMin = 0
originalMax = 1

class ParticipantMotion:
    def __init__(self, rigidbodyNum: int, mocapServer: str = '', mocapLocal: str = '', gripperNum: int = 1, bendingSensorSerialCOMs: list = []) -> None:

        self.rigidbodyNum            = rigidbodyNum
        self.gripperNum        = gripperNum
        self.InitGripSensorValues    = []

        # ----- Initialize participants' motion input system ----- #
        self.optiTrackStreamingManager = OptiTrackStreamingManager(rigidBodyNum = rigidbodyNum, mocapServer=mocapServer,mocapLocal=mocapLocal)

        # ----- Start streaming from OptiTrack ----- #
        streamingThread = threading.Thread(target=self.optiTrackStreamingManager.stream_run)
        streamingThread.setDaemon(True)
        streamingThread.start()

        # ----- Initialize gripper control system ----- #
        self.Port = bendingSensorSerialCOMs
        self.Bandrate = 115200

        self.gripperSensorManager = GripperSensorManager(ip=self.Port, port=self.Bandrate)

        # ----- Start receiving bending sensor value from UDP socket ----- #
        bendingSensorThread = threading.Thread(target = self.gripperSensorManager.StartReceiving)
        bendingSensorThread.setDaemon(True)
        bendingSensorThread.start()

        # ----- Set init value ----- #
        self.SetInitialBendingValue()

    def SetInitialBendingValue(self):
        """
        Set init bending value
        """
        
        self.InitBendingSensorValue = self.gripperSensorManager.sensorValue

    def LocalPosition(self, loopCount: int = 0):
        """
        Local position

        Parameters
        ----------
        loopCount: (Optional) int
            For recorded motion.
            Count of loop.

        Returns
        ----------
        participants' local position: dict
        {'participant1': [x, y, z]}
        unit: [m]
        """

        dictPos = self.optiTrackStreamingManager.position

        return dictPos

    def LocalRotation(self, loopCount: int = 0):
        """
        Local rotation

        Parameters
        ----------
        loopCount: (Optional) int
            For recorded motion.
            Count of loop.

        Returns
        ----------
        participants' local rotation: dict
        {'participant1': [x, y, z, w] or [x, y, z]}
        """

        dictRot = self.optiTrackStreamingManager.rotation

        return dictRot


    def GripperControlValue(self, loopCount: int = 0):
        """
        Value for control of the xArm gripper

        Parameters
        ----------
        loopCount: (Optional) int
            For recorded motion.
            Count of loop.

        Returns
        ----------`
        Value for control of the xArm gripper: dict
        {'gripperValue1': float value}
        """

        sensorVal = self.gripperSensorManager.bendingValue

        return sensorVal
