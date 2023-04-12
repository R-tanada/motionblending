# ----------------------------------------------------------------------- 
# Author:   Takayoshi Hagiwara (KMD)
# Created:  2021/8/20
# Summary:  Cybernetic avatar の運動制御マネージャー
# -----------------------------------------------------------------------

import numpy as np
import quaternion
import math

class AvatarMotion:

    originPositions     = {}
    inversedMatrixforPosition ={}
    inversedMatrix      = {}

    def __init__(self) -> None:
        self.originPositions = np.zeros(3)
        self.inversedMatrixforPosition = np.array([[1,0,0],[0,1,0],[0,0,1]])
        self.inversedMatrix = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    def GetAvatarMotion(self, position: dict, rotation: dict, isRelativePosition: bool = True, isRelativeRotation: bool = True):
        if type(position) is np.ndarray:
            position = self.NumpyArray2Dict(position)
        
        if type(rotation) is np.ndarray:
            rotation = self.NumpyArray2Dict(rotation)

        relativePosition = self.GetRelativePosition(position)
        relativeRotation = self.GetRelativeRotation(rotation)

        relativeRotation_euler = self.Quaternion2Euler(relativeRotation)
        
        return relativePosition, relativeRotation_euler
    

    def SetOriginPosition(self, position) -> None:
        # ----- numpy array to dict: position ----- #
        if type(position) is np.ndarray:
            position = self.NumpyArray2Dict(position)
        
        self.originPositions = position
        
    def GetRelativePosition(self, position):
        # ----- numpy array to dict: position ----- #
        if type(position) is np.ndarray:
            position = self.NumpyArray2Dict(position)
        
        relativePos = {}
        relativePos = position - self.originPositions
        
        return relativePos
    
    def SetInversedMatrix(self, rotation) -> None:
        # ----- numpy array to dict: rotation ----- #
        if type(rotation) is np.ndarray:
            rotation = self.NumpyArray2Dict(rotation)

        q = rotation
        qw, qx, qy, qz = q[3], q[1], q[2], q[0]
        mat4x4 = np.array([ [qw, -qy, qx, qz],
                            [qy, qw, -qz, qx],
                            [-qx, qz, qw, qy],
                            [-qz,-qx, -qy, qw]])
        self.inversedMatrix = np.linalg.inv(mat4x4)
    
    def GetRelativeRotation(self, rotation):
        # ----- numpy array to dict: rotation ----- #
        if type(rotation) is np.ndarray:
            rotation = self.NumpyArray2Dict(rotation)
        
        relativeRot = {}
        relativeRot = np.dot(self.inversedMatrix, rotation)

        return relativeRot
    
    def Quaternion2Euler(self, q, isDeg: bool = True):
        qx = q[0]
        qy = q[1]
        qz = q[2]
        qw = q[3]

        m00 = 1 - (2 * qy**2) - (2 * qz**2)
        m01 = (2 * qx * qy) + (2 * qw * qz)
        m02 = (2 * qx * qz) - (2 * qw * qy)
        m10 = (2 * qx * qy) - (2 * qw * qz)
        m11 = 1 - (2 * qx**2) - (2 * qz**2)
        m12 = (2 * qy * qz) + (2 * qw * qx)
        m20 = (2 * qx * qz) + (2 * qw * qy)
        m21 = (2 * qy * qz) - (2 * qw * qx)
        m22 = 1 - (2 * qx**2) - (2 * qy**2)

        if m02 == 1:
            tx = math.atan2(m10, m11)
            ty = math.pi/2
            tz = 0
        elif m02 == -1:
            tx = math.atan2(m21, m20)
            ty = -math.pi/2
            tz = 0
        else:
            tx = -math.atan2(-m12, m22)
            ty = -math.asin(m02)
            tz = -math.atan2(-m01, m00)

        if isDeg:
            tx = np.rad2deg(tx)
            ty = np.rad2deg(ty)
            tz = np.rad2deg(tz)

        rotEuler = np.array([tx, ty, tz])
        return rotEuler
    
    def Euler2Quaternion(self, e):
        roll = np.deg2rad(e[0])
        pitch = np.deg2rad(e[1])
        yaw = np.deg2rad(e[2])

        cosRoll = np.cos(roll/2.0)
        sinRoll = np.sin(roll / 2.0)
        cosPitch = np.cos(pitch / 2.0)
        sinPitch = np.sin(pitch / 2.0)
        cosYaw = np.cos(yaw / 2.0)
        sinYaw = np.sin(yaw / 2.0)

        q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw
        q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw
        q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw
        q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw

        rotQuat = [q1, q2, q3, q0]
        return rotQuat

    def NumpyArray2Dict(self, numpyArray, dictKey: str = 'participant'):
        if type(numpyArray) is np.ndarray:
            dictionary = {}
            if len(numpyArray.shape) == 1:
                dictionary[dictKey+str(1)] = numpyArray
            else:
                for i in range(len(numpyArray)):
                    dictionary[dictKey+str(i+1)] = numpyArray[i]
        else:
            print('Type Error: argument is NOT Numpy array')
            return
        
        return dictionary