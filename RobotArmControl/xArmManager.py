from RobotArmControl.SafetyManager import SafetyManager
from xarm.wrapper import XArmAPI


class xArmManager:
    def __init__(self, xArmConfig: dict) -> None:
        self.arm = XArmAPI(xArmConfig['IP'])
        self.InitializeAll(xArmConfig['InitPos'], xArmConfig['InitRot'])

    def DisConnect(self):
        self.arm.disconnect()

    def CheckError(self):
        if self.arm.has_err_warn:
            print('[ERROR] >> xArm Error has occured.')

    def SendDataToRobot(self, transform):
        self.arm.set_servo_cartesian()
        self.arm.getset_tgpio_modbus_data(self.ConvertToModbusData())

    def InitializeAll(self, InitPos, InitRot):
        self.arm.connect()
        if self.arm.warn_code != 0:
            self.arm.clean_warn()
        if self.arm.error_code != 0:
            self.arm.clean_error()
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)             # set mode: position control mode
        self.arm.set_state(state=0)      # set state: sport state

        self.arm.set_position(x = InitPos[0], y = InitPos[1], z = InitPos[2], roll = InitRot[0], pitch = InitRot[1], yaw = InitRot[2], wait=True)
        print('Initialized > xArm')

        self.arm.set_tgpio_modbus_baudrate(2000000)
        self.arm.set_gripper_mode(0)
        self.arm.set_gripper_enable(True)
        self.arm.set_gripper_position(850, speed=5000)
        self.arm.getset_tgpio_modbus_data(self.ConvertToModbusData(850))
        print('Initialized > xArm gripper')

        self.arm.set_mode(1)
        self.arm.set_state(state=0)

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

    def AddOffset(self, position, rotation):
        pass

    def CheckLimit(self):
        pass




