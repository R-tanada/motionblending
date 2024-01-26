import json
import random
import socket
import threading
import time

import numpy as np

import lib.self.CustomFunction as cf
from src.DataManager import DataLoadManager, DataPlotManager, DataRecordManager
from src.FeedbackManager import LED_Feedback, Vibrotactile
from src.MinimunJerk import MinimumJerk
from src.mode_select import mode, name
# # ----- Custom class ----- #
from src.OptiTrackStreamingManager import OptiTrackStreamingManager
from src.SensorManager import GripperSensorManager


class ParticipantManager:
    with open("docs/settings_dual.json", "r") as settings_file:
        settings = json.load(settings_file)
    xArmConfig = {}
    for xArm in settings["xArmConfigs"].keys():
        xArmConfig[settings["xArmConfigs"][xArm]["Mount"]] = settings["xArmConfigs"][
            xArm
        ]

    def __init__(self, ParticipantConfig: dict, is_Simulation, is_Recording) -> None:
        self.participantConfig = ParticipantConfig
        self.position = []
        self.rotation = []

        self.motionManagers = {}
        for Config in self.participantConfig:
            self.motionManagers[Config["Mount"]] = MotionManager(
                Config,
                ParticipantManager.xArmConfig[Config["Mount"]],
                is_Simulation,
                is_Recording,
            )

    def GetParticipantMotion(self):
        participantMotions = {}
        for Config in self.participantConfig:
            participantMotions[Config["Mount"]] = self.motionManagers[
                Config["Mount"]
            ].GetMotionData()

        return participantMotions

    def SetParticipantInitPosition(self):
        for Config in self.participantConfig:
            self.motionManagers[Config["Mount"]].SetInitPosition()

    def SetParticipantInitRotation(self):
        for Config in self.participantConfig:
            self.motionManagers[Config["Mount"]].SetInitRotation()

    def SetElaspedTime(self, elaspedTime):
        for Config in self.participantConfig:
            self.motionManagers[Config["Mount"]].SetElaspedTime(elaspedTime)

    def ExportCSV(self):
        for Config in self.participantConfig:
            self.motionManagers[Config["Mount"]].ExportCSV()

    def PlotGraph(self):
        for Config in self.participantConfig:
            self.motionManagers[Config["Mount"]].PlotGraph()


class MotionManager:
    optiTrackStreamingManager = OptiTrackStreamingManager(
        mocapServer="133.68.108.58", mocapLocal="133.68.108.58"
        # mocapServer="127.0.0.1",
        # mocapLocal="127.0.0.1",
    )
    streamingThread = threading.Thread(target=optiTrackStreamingManager.stream_run)
    streamingThread.setDaemon(True)
    streamingThread.start()

    target_index_right = []
    target_index_left = []

    while True:
        target_index_left = random.sample(range(0, 4), k=2)
        target_index_right = random.sample(range(0, 4), k=2)

        if (target_index_right[0] != target_index_left[0]) and (
            target_index_right[1] != target_index_left[1]
        ):
            break

        time.sleep(0.05)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        data_right = "right," + str(target_index_right[0] + 1)
        data_left = "left," + str(target_index_left[0] + 1)
        message_right = data_right.encode("utf-8")
        message_left = data_left.encode("utf-8")
        sock.sendto(message_right, ("127.0.0.1", 8888))
        sock.sendto(message_left, ("127.0.0.1", 8888))
        time.sleep(1)

    def __init__(self, Config, xArmConfig, is_Simulation, is_Recording) -> None:
        self.mount = Config["Mount"]
        self.rigidBody = str(Config["RigidBody"])
        self.weight = Config["Weight"]
        self.initPosition = []
        self.initQuaternion = []
        self.initInverseMatrix = []
        self.initFlag = False
        self.updateInitPosition = []
        self.updateInitQuaternion = []
        self.updateInitInverseMatrix = []
        self.iter_initPos = self.iter_initRot = []
        self.isMoving_Pos = (
            self.isMoving_Rot
        ) = self.isMoving_Grip = self.isMoving = False
        self.pos_list = []
        self.pos_list2 = []
        self.pos_list3 = []
        self.pos_box = []
        self.vel_list = []
        self.vel_box = []
        self.dt = 1 / 200
        self.before_time = 0
        self.recording = is_Recording
        self.Simulation = is_Simulation
        self.elaspedTime = 0
        self.auto_list = []
        self.grip_data = 850
        self.feedback_count = 255

        self.mode = mode
        print(mode)

        if mode == 4:
            self.fb_manager = LED_Feedback(Config["LED_serial"])
        elif mode == 3:
            self.fb_manager = Vibrotactile(Config["AudioIndex"])
        else:
            pass

        if self.mount == "right":
            target_index = MotionManager.target_index_right
        elif self.mount == "left":
            target_index = MotionManager.target_index_left
        self.automation = MinimumJerk(Config["Target"], xArmConfig, target_index)

        self.sensorManager = GripperSensorManager(Config["SerialCOM"], BandRate=115200)
        sensorThread = threading.Thread(target=self.sensorManager.StartReceiving)
        sensorThread.setDaemon(True)
        sensorThread.start()

        # self.recorder = DataPlotManager(legend = ['x_mocap', 'x_minimumjerk'], xlabel='time[s]', ylabel='position[mm]')
        if self.mode == 0:
            # self.recorder2 = DataRecordManager(header=['time', 'velocity'], fileName='velocity', custom=False)
            self.recorder = DataRecordManager(
                header=["time", "x", "y", "z"],
                fileName=name + "/" + "model_data/_" + self.mount,
                custom=True,
            )
            switch_thread = threading.Thread(target=self.recorder.key_thread)
            switch_thread.setDaemon(True)
            switch_thread.start()
        # self.recorder = DataPlotManager(legend = ['x_robot'], xlabel='time[s]', ylabel='position[mm]')

        elif self.mode == 1 or self.mode == 2 or self.mode == 3 or self.mode == 4:
            self.recorder_user = DataRecordManager(
                header=["x", "y", "z"],
                fileName=name + "/" + "user_data/" + str(self.mode) + "_" + self.mount,
            )
            self.recorder_robot = DataRecordManager(
                header=["x", "y", "z"],
                fileName=name + "/" + "robot_data/" + str(self.mode) + "_" + self.mount,
            )
            self.recorder_traj = DataRecordManager(
                header=["x", "y", "z"],
                fileName=name + "/" + "auto_data/" + str(self.mode) + "_" + self.mount,
            )
            self.recorder_time = DataRecordManager(
                header=["x", "y", "z"],
                fileName=name + "/" + "time_data/" + str(self.mode) + "_" + self.mount,
            )

        if self.Simulation:
            self.data_pos = DataLoadManager(Config["DataPath"]["position"])
            self.data_rot = DataLoadManager(Config["DataPath"]["rotation"])
            self.data_grip = DataLoadManager(Config["DataPath"]["gripper"])
            self.data_time = DataLoadManager(Config["DataPath"]["time"])

        else:
            MotionManager.optiTrackStreamingManager.position[self.rigidBody] = np.zeros(
                3
            )
            MotionManager.optiTrackStreamingManager.rotation[self.rigidBody] = np.zeros(
                4
            )

    def GetMotionData(self):
        position, rotation, gripper = (
            self.GetPosition(),
            self.GetRotation(),
            self.GetGripperValue(),
        )
        if self.mode == 0:
            self.recorder.custom_record(
                np.hstack((self.elaspedTime, position)), self.grip_data, self.mount
            )
        velocity, accelaration = self.GetParticipnatMotionInfo(position)

        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            if self.isMoving == True:
                self.UpdateInitPosition(position)
                self.UpdateInitRotation(rotation)
                self.isMoving = False

            if self.isMoving == False and self.initFlag == True:
                posFlag = self.LerpInitPosition()
                rotFlag = self.SlerpInitRotation()
                if posFlag == rotFlag == False:
                    print("finish_automation: " + self.mount)
                    self.initFlag = False

                    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                        if self.mount == "right":
                            data = "right," + str(
                                MotionManager.target_index_right[1] + 1
                            )
                        elif self.mount == "left":
                            data = "left," + str(MotionManager.target_index_left[1] + 1)
                        message = data.encode("utf-8")
                        sock.sendto(message, ("127.0.0.1", 8888))

            if self.initFlag == False:
                if self.automation.MonitoringMotion(
                    position, rotation, gripper, velocity, accelaration
                ):
                    self.isMoving_Pos = (
                        self.isMoving_Rot
                    ) = self.isMoving_Grip = self.isMoving = self.initFlag = True

        return {
            "position": position,
            "rotation": rotation,
            "gripper": gripper,
            "weight": self.weight,
        }

    def GetPosition(self):
        if self.Simulation:
            position = self.data_pos.getdata()
        else:
            position = MotionManager.optiTrackStreamingManager.position[self.rigidBody]

        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            self.position = cf.ConvertAxis_Position(
                position * 1000, self.mount
            ) - np.array(self.initPosition)

            if self.mode == 1 or self.mode == 2 or self.mode == 3 or self.mode == 4:
                self.recorder_user.record(self.position)
                self.recorder_robot.record(self.position)
                self.recorder_traj.record([0, 0, 0])

        else:
            (
                pos_auto,
                self.isMoving_Pos,
                weight,
                velocity_auto,
            ) = self.automation.GetPosition(self.elaspedTime)
            position = cf.ConvertAxis_Position(position * 1000, self.mount) - np.array(
                self.initPosition
            )
            self.position = pos_auto * weight + position * (1 - weight)

            if mode == 3 or mode == 4:
                self.fb_manager.data_out = int(weight * 255)

            if self.mode == 1 or self.mode == 2 or self.mode == 3 or self.mode == 4:
                self.recorder_user.record(position)
                self.recorder_robot.record(self.position)
                self.recorder_traj.record(pos_auto)

            # self.recorder.record(np.hstack([position[0], pos_auto[0],  self.elaspedTime]))

        return self.position

    def GetRotation(self):
        if self.Simulation:
            rotation = self.data_rot.getdata()
        else:
            rotation = MotionManager.optiTrackStreamingManager.rotation[self.rigidBody]
            # if self.recording:
            #     self.recorder_rot.record(rotation)

        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            quaternion = cf.CnvertAxis_Rotation(rotation, self.mount)
            if quaternion[3] < 0:
                quaternion = -np.array(quaternion)
            self.rotation = quaternion
        else:
            rot_auto, self.isMoving_Rot, weight = self.automation.GetRotation(
                self.elaspedTime
            )
            quaternion = cf.CnvertAxis_Rotation(rotation, self.mount)
            if quaternion[3] < 0:
                quaternion = -np.array(quaternion)
            self.rotation = cf.Slerp_Quaternion(rot_auto, quaternion, weight)
            # print(self.rotation, rot_auto)
            # print(weight)

        return [self.rotation, self.initQuaternion, self.initInverseMatrix]

    def GetGripperValue(self):
        if self.Simulation:
            grip = self.data_grip.getdata()[0]
        else:
            grip = self.grip_data = cf.ConvertSensorToGripper(
                self.sensorManager.sensorValue
            )
            # if self.recording:
            #     self.recorder_grip.record([grip])

        if self.isMoving_Pos == self.isMoving_Rot == self.isMoving_Grip == False:
            gripper = grip
        else:
            gripper, self.isMoving_Grip = self.automation.GetGripperValue()

        return gripper

    def SetInitPosition(self):
        if self.Simulation:
            initPosition = self.data_pos.getdata()
        else:
            initPosition = MotionManager.optiTrackStreamingManager.position[
                self.rigidBody
            ]
            # if self.recording:
            #     self.recorder_pos.record(initPosition)

        self.initPosition = cf.ConvertAxis_Position(initPosition * 1000, self.mount)

    def SetInitRotation(self):
        if self.Simulation:
            initQuaternion = self.data_rot.getdata()
        else:
            initQuaternion = MotionManager.optiTrackStreamingManager.rotation[
                self.rigidBody
            ]
            # if self.recording:
            #     self.recorder_rot.record(initQuaternion)

        quaternion = cf.CnvertAxis_Rotation(initQuaternion, self.mount)
        if quaternion[3] < 0:
            quaternion = -np.array(quaternion)
        self.initQuaternion = self.automation.q_init = quaternion
        self.initInverseMatrix = cf.Convert2InverseMatrix(quaternion)

    def UpdateInitPosition(self, position):
        self.updateInitPosition = self.initPosition - (
            np.array(position) - self.GetPosition()
        )
        p_list = np.linspace(self.updateInitPosition, self.initPosition, 300)
        self.iter_initPos = iter(p_list)

    def UpdateInitRotation(self, rotation):
        q_zero = [0, 0, 0, 1]
        quaternion, initQuaternion, initInveseMatrix = self.GetRotation()
        q_inverse = np.dot(cf.Convert2InverseMatrix(quaternion), q_zero)
        self.updateInitQuaternion = np.dot(
            initInveseMatrix, np.dot(cf.Convert2Matrix(rotation[0]), q_inverse)
        )
        self.updateInitQuaternion = np.dot(
            cf.Convert2InverseMatrix(self.updateInitQuaternion), q_zero
        )
        weight_list = np.linspace(0, 1, 300)
        q_list = []
        for weight in weight_list:
            q_list.append(
                cf.Slerp_Quaternion(
                    self.initQuaternion, self.updateInitQuaternion, weight
                )
            )

        self.iter_initRot = iter(q_list)

    def LerpInitPosition(self):
        try:
            self.initPosition, flag = next(self.iter_initPos), True

            if mode == 3 or mode == 4:
                self.feedback_count -= 255 / 300
                self.fb_manager.data_out = int(self.feedback_count)
        except StopIteration:
            flag = False
            if mode == 3 or mode == 4:
                self.feedback_count = 255

        return flag

    def SlerpInitRotation(self):
        try:
            rot = next(self.iter_initRot)
            # print(rot)
            self.initQuaternion, self.initInverseMatrix, flag = (
                rot,
                cf.Convert2InverseMatrix(rot),
                True,
            )
        except StopIteration:
            flag = False
            self.initQuaternion = self.automation.q_init
            self.initInverseMatrix = cf.Convert2InverseMatrix(self.automation.q_init)

        return flag

    def GetParticipnatMotionInfo(self, position, interval=25):
        self.pos_list.append(position)

        if len(self.pos_list) == interval + 1:
            vel = np.linalg.norm(
                np.polyfit(
                    np.linspace(0, self.dt * (interval + 1), (interval + 1)),
                    self.pos_list,
                    1,
                )[0]
            )
            del self.pos_list[0]

        else:
            vel = 0

        # self.recorder2.custom_record(np.hstack((self.elaspedTime, [vel])))

        # print(vel)

        return vel, 0

    def GetParticipnatMotionInfo2(self, position, interval=25):
        self.pos_list2.append(position)

        if len(self.pos_list2) == interval + 1:
            vel = np.linalg.norm(
                np.polyfit(
                    np.linspace(0, self.dt * (interval + 1), (interval + 1)),
                    self.pos_list2,
                    1,
                )[0]
            )
            del self.pos_list2[0]

        else:
            vel = 0

        # self.recorder2.record(np.hstack(([vel], self.elaspedTime)))

        # print(vel)

        return vel, 0

    def GetParticipnatMotionInfo3(self, position, interval=25):
        self.pos_list3.append(position)

        if len(self.pos_list3) == interval + 1:
            vel = np.linalg.norm(
                np.polyfit(
                    np.linspace(0, self.dt * (interval + 1), (interval + 1)),
                    self.pos_list3,
                    1,
                )[0]
            )
            del self.pos_list3[0]

        else:
            vel = 0

        # self.recorder2.record(np.hstack(([vel], self.elaspedTime)))

        # print(vel)

        return vel, 0

    def ExportCSV(self):
        # self.recorder_pos.exportAsCSV()
        # self.recorder_rot.exportAsCSV()
        # self.recorder_grip.exportAsCSV()
        # self.recorder_time.exportAsCSV()
        # self.recorder2.exportAsCSV()
        # self.recorder.exportAsCSV()
        self.recorder_robot.exportAsCSV()
        self.recorder_user.exportAsCSV()
        self.recorder_traj.exportAsCSV()
        self.recorder_time.exportAsCSV()

    def PlotGraph(self):
        pass
        # self.recorder.plotGraph()
        # self.recorder3.plotGraph()

    def SetElaspedTime(self, elaspedTime):
        if self.Simulation == True:
            self.elaspedTime = self.data_time.getdata()[0]

        else:
            self.elaspedTime = elaspedTime
            if self.mode == 1 or self.mode == 2 or self.mode == 3 or self.mode == 4:
                self.recorder_time.record([self.elaspedTime])
