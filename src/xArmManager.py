from RobotArmControl.SafetyManager import SafetyManager
from xarm.wrapper import XArmAPI


class xArmManager:
    def __init__(self, xArmConfigs: dict) -> None:
        self.Arms = {}
        self.safetyManagers = {}
        for xArm in xArmConfigs.keys():
            self.Arms[xArmConfigs[xArm]['Mount']] = XArmAPI(xArmConfigs[xArm]['IP'])
            self.safetyManagers[xArmConfigs[xArm]['Mount']] = SafetyManager(xArmConfigs[xArm])
            self.InitializeAll(self.Arms[xArmConfigs[xArm]['Mount']], xArmConfigs[xArm]['InitPos'], xArmConfigs[xArm]['InitRot'])

    def DisConnect(self):
        for mount in self.Arms.keys():
            self.Arms[mount].disconnect()

    def CheckError(self):
        for mount in self.Arms.keys():
            if self.Arms[mount].has_err_warn:
                print('[ERROR] >> xArm Error has occured.')

    def SendDataToRobot(self, transform):
        for mount in self.Arms.keys():
            self.Arms[mount].set_servo_cartesian(self.safetyManagers[mount].CheckLimit(transform[mount]['position'], transform[mount]['rotation']))
            self.Arms[mount].getset_tgpio_modbus_data(self.ConvertToModbusData(transform[mount]['gripper']))

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

    def InitializeAll(self, Arm, InitPos, InitRot):
        Arm.connect()
        if Arm.warn_code != 0:
            Arm.clean_warn()
        if Arm.error_code != 0:
            Arm.clean_error()
        Arm.motion_enable(enable=True)
        Arm.set_mode(0)             # set mode: position control mode
        Arm.set_state(state=0)      # set state: sport state

        Arm.set_position(x = InitPos[0], y = InitPos[1], z = InitPos[2], roll = InitRot[0], pitch = InitRot[1], yaw = InitRot[2], wait=True)
        print('Initialized > xArm')

        Arm.set_tgpio_modbus_baudrate(2000000)
        Arm.set_gripper_mode(0)
        Arm.set_gripper_enable(True)
        Arm.set_gripper_position(850, speed=5000)
        Arm.getset_tgpio_modbus_data(self.ConvertToModbusData(850))
        print('Initialized > xArm gripper')

        Arm.set_mode(1)
        Arm.set_state(state=0)

    def MergeOffset(self, position, rotation):
        pass

    def CheckLimit(self):
        pass




