import os
import sys
import time
import json

from xarm.wrapper import XArmAPI

ip = '192.168.1.240'

arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)
time.sleep(1)

transform = arm.get_position()[1]

with open('SettingFile/settings_single.json') as f:
    setting_update = json.load(f)
    setting_update['ParticipantsConfigs']['participant1'][0]['Target'][0]['position'] = transform[0:3]
    setting_update['ParticipantsConfigs']['participant1'][0]['Target'][0]['rotation'] = transform[3:6]
    json.dump(setting_update, f)

arm.disconnect()
