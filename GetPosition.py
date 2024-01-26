import json
import os
import sys
import time

from lib.xarm.wrapper import XArmAPI

arm = "left"
target = 1


if arm == "right":
    num = 0
    ip = "192.168.1.199"
elif arm == "left":
    num = 1
    ip = "192.168.1.228"
target -= 1

arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
time.sleep(1)

transform = arm.get_position()[1]

with open("docs\settings_dual.json") as f:
    setting_update = json.load(f)
    setting_update["ParticipantsConfigs"]["participant1"][num]["Target"][target][
        "position"
    ] = transform[0:3]
    setting_update["ParticipantsConfigs"]["participant1"][num]["Target"][target][
        "rotation"
    ] = transform[3:6]

with open("docs\settings_dual.json", "w") as f:
    json.dump(setting_update, f)

arm.disconnect()
