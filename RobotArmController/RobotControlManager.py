import json
import time
from ctypes import windll

import numpy as np
from Sensor.SensorManager import GripperSensorManager
from CyberneticAvatarMotion.CyberneticAvatarMotionManager import CyberneticAvatarMotionManager
from matplotlib.pyplot import flag
# ----- Motion Function -----#
from ParticipantMotion.ParticipantMotionManager import ParticipantMotionManager
# ----- Custom class ----- #
from RobotArmController.xArmTransform import xArmTransform
from xarm.wrapper import XArmAPI



class RobotControlManagerClass:
    def __init__(self) -> None:
        with open('settings.json', 'r') as setting_file:
            settings_data = json.load(setting_file)

        self.xArmConfigs = settings_data['xArmConfigs']
        self.SensorSerialCOMs = settings_data['SensorSerialCOMs']
        self.ParticipantConfigs = settings_data['ParticipantConfigs']

    def SendDataToRobot(self, executionTime: int = 9999, isFixedFrameRate: bool = False, frameRate: int = 90, isChangeOSTimer: bool = False, isExportData: bool = True, isEnablexArm: bool = True):
        # ----- Change OS timer ----- #
        if isFixedFrameRate and isChangeOSTimer:
            windll.winmm.timeBeginPeriod(1)

        # ----- Process info ----- #
        self.loopCount      = 0
        self.taskTime       = []
        taskStartTime       = time.perf_counter()

        # ----- Set loop time from frameRate ----- #
        loopTime        = 1 / frameRate
        loopStartTime   = 0
        processDuration = 0
        listFrameRate   = []
        if isFixedFrameRate:
            print('Use fixed frame rate > ' + str(frameRate) + '[fps]')


        # ----- Instantiating custom classes ----- #
        CA_Manager                         = CyberneticAvatarMotionManager(defaultParticipantNum = participantNum)
        transform                          = xArmTransform()
        participantManagers = []
        for participantConfig in self.ParticipantConfigs:
            participantManagers.append(ParticipantMotionManager(participantConfig))

        # ----- Initialize robot arm ----- #
        if isEnablexArm:
            arms = {}
            for xArmConfig in self.xArmConfigs:
                arms[xArmConfig['Mount']] = XArmAPI(xArmConfig['IP'])
                self.InitializeAll(arm, transform)

        # ----- Control flags ----- #
        isMoving    = False

        try:
            while True:
                if isMoving:
                    # ---------- Start control process timer ---------- #
                    loopStartTime = time.perf_counter()

                    # ----- Get participant motions ----- #
                    for i in len(participantManagers):
                        participantMotions[i] = participantManagers[i].GetParticipantMotion()

                    participantMotions = [
                        {
                        'right': {'position': [1, 1, 1], 'rotation': [1, 1, 1], 'gripper': 5}, 
                        'left': {'position': [1, 1, 1], 'rotation': [1, 1, 1], 'gripper': 5}
                        }, 
                        {
                        'right': {'position': [1, 1, 1], 'rotation': [1, 1, 1], 'gripper': 5}
                        }
                    ]

                    transform = CA_Manager.GetSharedTransform(participantMotions, weightSliderList)

                    transform = {
                                'left': {'position': [0, 0, 0], 'rotation': [0, 0, 0], 'gripper': 0},
                                'right': {'position': [0, 0, 0], 'rotation': [0, 0, 0], 'gripper': 0}
                                }  

                    if isEnablexArm:
                        for xArmConfig in self.xArmConfigs:
                            arms[xArmConfig['Mount']].set_servo_cartesian([transform['position'],transform['rotation']])
                            arms[xArmConfig['Mount']].getset_tgpio_modbus_data(self.ConvertToModbusData(transform['gripper']))

                    # ----- If xArm error has occured ----- #
                    for arm in arms:
                        if isEnablexArm and arm.has_err_warn:
                            isMoving    = False
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

                    self.loopCount += 1

                else:
                    keycode = input('Input > "q": quit, "r": Clean error and init arm, "s": start control \n')
                    # ----- Quit program ----- #
                    if keycode == 'q':
                        if isEnablexArm:
                            for xArmConfig in self.xArmConfigs:
                                arms[xArmConfig['Mount']].disconnect()
                        self.PrintProcessInfo()

                        windll.winmm.timeEndPeriod(1)
                        break

                    # ----- Reset xArm and gripper ----- #
                    elif keycode == 'r':
                        if isEnablexArm:
                            self.InitializeAll(arm, transform)

                    # ----- Start streaming ----- #
                    elif keycode == 's':
                        Behaviour.SetOriginPosition(participantMotionManager.LocalPosition())
                        caBehaviour.SetInversedMatrix(participantMotionManager.LocalRotation())

                        participantMotionManager.SetInitialBendingValue()

                        isMoving    = True
                        taskStartTime = time.perf_counter()

        except KeyboardInterrupt:
            print('\nKeyboardInterrupt >> Stop: RobotControlManager.SendDataToRobot()')

            self.taskTime.append(time.perf_counter() - taskStartTime)
            self.PrintProcessInfo()

            # Graph2DManager.soloy_graph()

            if isExportData:
                dataRecordManager.ExportSelf(dirPath='Recorder/RecordedData/motion_noise')

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
