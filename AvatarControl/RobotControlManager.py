# -----------------------------------------------------------------------
# Author:   Takayoshi Hagiwara (KMD)
# Created:  2021/8/19
# Summary:  Robot arm motion control manager
# -----------------------------------------------------------------------

import datetime
import json
import threading
import time
from ctypes import windll
from datetime import datetime
from enum import Flag

import numpy as np
from BendingSensorManager import BendingSensorManager
from CyberneticAvatarMotionBehaviour import \
    CyberneticAvatarMotionBehaviour
from FileIO.FileIO import FileIO
from matplotlib.pyplot import flag
# ----- Motion Function -----#
from avatar.ParticipantMotionManager import ParticipantMotionManager
from Recorder.DataRecordManager import DataRecordManager
from RobotArmController.WeightSliderManager import WeightSliderManager
# ----- Custom class ----- #
from RobotArmController.xArmTransform import xArmTransform
from VibrotactileFeedback.VibrotactileFeedbackManager import \
    VibrotactileFeedbackManager
from xarm.wrapper import XArmAPI

# ----- Safety settings. Unit: [mm] ----- #
movingDifferenceLimit = 1000

class RobotControlManagerClass:
    def __init__(self) -> None:
        setting_file = open('settings.json', 'r')
        settings_data = json.load(setting_file)

        self.xArmIP = settings_data['xArmIP']
        self.MotiveServerIP = settings_data['MotiveServerIP']
        self.MotiveLocalIP = settings_data['MotiveLocalIP']
        self.BendingSensorCOMs = settings_data['BendingSensorCOM']

    def SendDataToRobot(self, participantNum, executionTime: int = 9999, isFixedFrameRate: bool = False, frameRate: int = 90, isChangeOSTimer: bool = False, isExportData: bool = True, isEnablexArm: bool = True):

        # ----- Change OS timer ----- #
        if isFixedFrameRate and isChangeOSTimer:
            windll.winmm.timeBeginPeriod(1)

        # ----- Process info ----- #
        self.loopCount      = 0
        self.taskTime       = []
        self.errorCount     = 0
        taskStartTime       = time.perf_counter()

        # ----- Set loop time from frameRate ----- #
        loopTime        = 1 / frameRate
        loopStartTime   = 0
        processDuration = 0
        listFrameRate   = []
        if isFixedFrameRate:
            print('Use fixed frame rate > ' + str(frameRate) + '[fps]')


        # ----- Instantiating custom classes ----- #
        caBehaviour                         = CyberneticAvatarMotionBehaviour(defaultParticipantNum=participantNum)
        transform                           = xArmTransform()
        dataRecordManager                   = DataRecordManager(participantNum=participantNum, otherRigidBodyNum=otherRigidBodyCount)
        participantMotionManager            = ParticipantMotionManager(defaultParticipantNum=participantNum, recordedParticipantNum=recordedParticipantMotionCount, motionInputSystem=motionDataInputMode,mocapServer=self.MotiveServerIP,mocapLocal=self.MotiveLocalIP,
                                                                       gripperInputSystem=gripperDataInputMode, bendingSensorNum=self.BendingSensorNum, recordedGripperValueNum=recordedGripperValueCount,
                                                                       BendingSensor_ConnectionMethod='wired',bendingSensorSerialCOMs=self.BendingSensorCOMs)
        # weightSliderManager                 = WeightSliderManager(WeightSlider_ConnectionMethod='wireless',ip=self.wirelessIpAddress,port=self.weightSliderPort)
        # vibrotactileFeedbackManager         = VibrotactileFeedbackManager()

        # ----- Initialize robot arm ----- #
        if isEnablexArm:
            arm = XArmAPI(self.xArmIP)
            self.InitializeAll(arm, transform)

        # ----- Control flags ----- #
        isMoving    = False

        # ----- Internal flags ----- #
        isPrintFrameRate    = False     # For debug
        isPrintData         = False     # For debug


        try:
            while True:
                if isMoving:
                    if time.perf_counter() - taskStartTime > executionTime:
                        # ----- Exit processing after task time elapses ----- #
                        isMoving    = False

                        self.taskTime.append(time.perf_counter() - taskStartTime)
                        self.PrintProcessInfo()

                        # ----- Export recorded data ----- #
                        if isExportData:
                            dataRecordManager.ExportSelf(dirPath='C:/Users/SANOLAB/Documents/GitHub/ms-proj-sharedavatar/Experiment1/Python/Recorder/RecordedData/2023_04_11',participant=self.participantname,conditions=self.condition)

                        # ----- Disconnect ----- #
                        if isEnablexArm:
                            arm.disconnect()

                        windll.winmm.timeEndPeriod(1)

                        print('----- Finish task -----')
                        break

                    # ---------- Start control process timer ---------- #
                    loopStartTime = time.perf_counter()

                    # ----- Bending sensor ----- #
                    dictBendingValue = participantMotionManager.GripperControlValue(loopCount=self.loopCount)

                    # ----- Get transform data----- #
                    localPosition = participantMotionManager.LocalPosition(loopCount=self.loopCount)
                    localRotation = participantMotionManager.LocalRotation(loopCount=self.loopCount)

                    position,rotation = caBehaviour.GetSharedTransformWithCustomWeight(localPosition, localRotation, weightSliderList)

                    position = position * 1000

                    # ----- coordinate transform ----- #
                    position_tr, rotation_tr = [0] * 3, [0] * 3
                    position_tr[0], position_tr[1], position_tr[2]  = position[2], position[0], position[1]
                    rotation_tr[0], rotation_tr[1], rotation_tr[2]  = rotation[2], rotation[0], rotation[1]

                    # ------ coordinate transform with right mounted ------#
                    # position_tr, rotation_tr = [0] * 3, [0] * 3
                    # position_tr[0], position_tr[1], position_tr[2]  = position[2], position[1], -1 * position[0]
                    # rotation_tr[0], rotation_tr[1], rotation_tr[2]  = rotation[1], -1 * rotation[2], -1 * rotation[0]

                    # ----- Set xArm transform ----- #
                    transform.x, transform.y, transform.z = position_tr[0], position_tr[1], position_tr[2]
                    transform.roll, transform.pitch, transform.yaw = rotation_tr[0], rotation_tr[1], rotation_tr[2]

                    # ----- Safety check (Position) ---- #
                    diffX = transform.x - beforeX
                    diffY = transform.y - beforeY
                    diffZ = transform.z - beforeZ
                    beforeX, beforeY, beforeZ = transform.x, transform.y, transform.z
                            
                    # ----- robot control ----- #
                    arm.set_servo_cartesian(transform.Transform(isLimit=True,isOnlyPosition=False))
                    code, ret = arm.getset_tgpio_modbus_data(self.ConvertToModbusData(gripperValue))

                    # ----- Data recording ----- #
                    dataRecordManager.Record(position_tr, rotation_tr, gripperValue, time.perf_counter()-taskStartTime)

                    # ----- If xArm error has occured ----- #
                    if isEnablexArm and arm.has_err_warn:
                        isMoving    = False
                        self.errorCount += 1
                        self.taskTime.append(time.perf_counter() - taskStartTime)
                        print('[ERROR] >> xArm Error has occured. Please enter "r" to reset xArm, or "q" to quit')

                     # ---------- End control process timer ---------- #
                    processDuration = time.perf_counter() - loopStartTime  # For loop timer

                    # ----- Fixed frame rate ----- #
                    if isFixedFrameRate:
                        sleepTime = loopTime - processDuration
                        if sleepTime < 0:
                            pass
                        else:
                            time.sleep(sleepTime)

                    # ----- (Optional) Check frame rate ----- #
                    if self.loopCount % 20 == 0 and isPrintFrameRate:
                        if self.loopCount != 0:
                            listFrameRate.append(1 / (time.perf_counter() - loopStartTime))
                            print("Average FPS: ", sum(listFrameRate)/len(listFrameRate))

                    # ----- (Optional) Check data ----- #
                    if isPrintData:
                        print('xArm transform > ' + str(np.round(transform.Transform(), 1)) + '   Bending sensor > ' + str(dictBendingValue))

                    self.loopCount += 1

                else:
                    keycode = input('Input > "q": quit, "r": Clean error and init arm, "s": start control \n')
                    # ----- Quit program ----- #
                    if keycode == 'q':
                        if isEnablexArm:
                            arm.disconnect()
                        self.PrintProcessInfo()

                        windll.winmm.timeEndPeriod(1)
                        break

                    # ----- Reset xArm and gripper ----- #
                    elif keycode == 'r':
                        if isEnablexArm:
                            self.InitializeAll(arm, transform)
                            # self.InitRobotArm(arm, transform)
                            # self.InitGripper(arm)

                    # ----- Start streaming ----- #
                    elif keycode == 's':
                        caBehaviour.SetOriginPosition(participantMotionManager.LocalPosition())
                        caBehaviour.SetInversedMatrix(participantMotionManager.LocalRotation())
                        participantMotionManager.SetInitialBendingValue()

                        position, rotation = caBehaviour.GetSharedTransformWithCustomWeight(participantMotionManager.LocalPosition(), participantMotionManager.LocalRotation(),weightSliderList )
                        beforeX, beforeY, beforeZ = position[2], position[0], position[1]

                        isMoving    = True
                        taskStartTime = time.perf_counter()

        except KeyboardInterrupt:
            print('\nKeyboardInterrupt >> Stop: RobotControlManager.SendDataToRobot()')

            self.taskTime.append(time.perf_counter() - taskStartTime)
            self.PrintProcessInfo()

            # Graph2DManager.soloy_graph()

            if isExportData:
                dataRecordManager.ExportSelf(dirPath='C:/Users/SANOLAB/Documents/GitHub/ms-proj-sharedavatar/Experiment1/Python/Recorder/RecordedData')

            # ----- Disconnect ----- #
            if isEnablexArm:
                arm.disconnect()

            windll.winmm.timeEndPeriod(1)

        except:
            print('----- Exception has occurred -----')
            windll.winmm.timeEndPeriod(1)
            import traceback
            traceback.print_exc()

    def ConvertToModbusData(self, value: int):
        if int(value) <= 255 and int(value) >= 0:
            dataHexThirdOrder = 0x00
            dataHexAdjustedValue = int(value)

        elif int(value) > 255 and int(value) <= 511:
            dataHexThirdOrder = 0x01
            dataHexAdjustedValue = int(value)-256

        elif int(value) > 511 and int(value) <= 767:
            dataHexThirdOrder = 0x02
            dataHexAdjustedValue = int(value)-512

        elif int(value) > 767 and int(value) <= 1123:
            dataHexThirdOrder = 0x03
            dataHexAdjustedValue = int(value)-768

        modbus_data = [0x08, 0x10, 0x07, 0x00, 0x00, 0x02, 0x04, 0x00, 0x00]
        modbus_data.append(dataHexThirdOrder)
        modbus_data.append(dataHexAdjustedValue)

        return modbus_data

    def PrintProcessInfo(self):
        print('----- Process info -----')
        print('Total loop count > ', self.loopCount)
        for ttask in self.taskTime:
            print('Task time\t > ', ttask, '[s]')
        print('Error count\t > ', self.errorCount)
        print('------------------------')

    def InitializeAll(self, robotArm, transform, isSetInitPosition=True):
        """
        Initialize the xArm

        Parameters
        ----------
        robotArm: XArmAPI
            XArmAPI object.
        transform: xArmTransform
            xArmTransform object.
        isSetInitPosition: (Optional) bool
            True -> Set to "INITIAL POSITION" of the xArm studio
            False -> Set to "ZERO POSITION" of the xArm studio
        """

        robotArm.connect()
        if robotArm.warn_code != 0:
            robotArm.clean_warn()
        if robotArm.error_code != 0:
            robotArm.clean_error()
        robotArm.motion_enable(enable=True)
        robotArm.set_mode(0)             # set mode: position control mode
        robotArm.set_state(state=0)      # set state: sport state
        if isSetInitPosition:
            initX, initY, initZ, initRoll, initPitch, initYaw = transform.GetInitialTransform()
            robotArm.set_position(x=initX, y=initY, z=initZ, roll=initRoll, pitch=initPitch, yaw=initYaw, wait=True)
        else:
            robotArm.reset(wait=True)
        print('Initialized > xArm')

        robotArm.set_tgpio_modbus_baudrate(2000000)
        robotArm.set_gripper_mode(0)
        robotArm.set_gripper_enable(True)
        robotArm.set_gripper_position(850, speed=5000)
        robotArm.getset_tgpio_modbus_data(self.ConvertToModbusData(850))
        print('Initialized > xArm gripper')

        robotArm.set_mode(1)
        robotArm.set_state(state=0)
