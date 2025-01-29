import time
import numpy as np
import sys
#sys.path.append("/home/habibi/DFKI/avp_teleoperate/unitree_sdk2_python")

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize


#find out what H1 with hands need
from unitree_sdk2py.idl.default import unitree_h1_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_h1.msg.dds_ import LowCmd_

from unitree_sdk2_python.unitree_sdk2py.utils.crc import CRC

NUM_ARM_MOTORS = 14


PUBBLISH_RATE = 30.0

def main():
    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    crc = CRC()

    cmd = unitree_h1_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0

