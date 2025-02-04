import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

# Adapted from h1_low_level_example.py
class H1JointIndex:
    # Right leg
    kRightHipYaw = 8
    kRightHipRoll = 0
    kRightHipPitch = 1
    kRightKnee = 2
    kRightAnkle = 11

    # Left leg
    kLeftHipYaw = 7
    kLeftHipRoll = 3
    kLeftHipPitch = 4
    kLeftKnee = 5
    kLeftAnkle = 10

    kWaistYaw = 6

    kNotUsedJoint = 9

    # Right arm
    kRightShoulderPitch = 12
    kRightShoulderRoll = 13
    kRightShoulderYaw = 14
    kRightElbow = 15

    # Left arm
    kLeftShoulderPitch = 16
    kLeftShoulderRoll = 17
    kLeftShoulderYaw = 18
    kLeftElbow = 19

NUM_MOTORS_H1 = 20

#########################################################

# Mapping to the H1JointIndex above.
default_joint_positions = np.zeros(NUM_MOTORS_H1, dtype=float)

# Left leg
default_joint_positions[H1JointIndex.kLeftHipYaw]   = 0.0
default_joint_positions[H1JointIndex.kLeftHipRoll]  = 0.0
default_joint_positions[H1JointIndex.kLeftHipPitch] = -0.1
default_joint_positions[H1JointIndex.kLeftKnee]     = 0.3
default_joint_positions[H1JointIndex.kLeftAnkle]    = -0.28

# Right leg
default_joint_positions[H1JointIndex.kRightHipYaw]  = 0.0
default_joint_positions[H1JointIndex.kRightHipRoll] = 0.0
default_joint_positions[H1JointIndex.kRightHipPitch]= -0.1
default_joint_positions[H1JointIndex.kRightKnee]    = 0.3
default_joint_positions[H1JointIndex.kRightAnkle]   = -0.28

# Torso
default_joint_positions[H1JointIndex.kWaistYaw]     = 0.0

# Left arm
default_joint_positions[H1JointIndex.kLeftShoulderPitch] = 0.0
default_joint_positions[H1JointIndex.kLeftShoulderRoll]  = 0.0
default_joint_positions[H1JointIndex.kLeftShoulderYaw]   = 0.0
default_joint_positions[H1JointIndex.kLeftElbow]         = 0.0

# Right arm
default_joint_positions[H1JointIndex.kRightShoulderPitch] = 0.0
default_joint_positions[H1JointIndex.kRightShoulderRoll]  = 0.0
default_joint_positions[H1JointIndex.kRightShoulderYaw]   = 0.0
default_joint_positions[H1JointIndex.kRightElbow]         = 0.0






def main():
    #input("Press Enter to continue...")

    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    cmd = unitree_go_msg_dds__LowCmd_()
    crc = CRC()

    # Basic header config
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0

    # Put motors in position-control mode
    # For "strong" motors, H1 uses mode 0x0A; for "weak" motors, mode 0x01
    # But for simplicity, let's just put everything in 0x0A. You can refine if needed.
    for i in range(NUM_MOTORS_H1):
        cmd.motor_cmd[i].mode = 0x0A  # position mode
        if i == H1JointIndex.kRightAnkle or i == H1JointIndex.kLeftAnkle:
            cmd.motor_cmd[i].kp = 300.0
            cmd.motor_cmd[i].kd = 8.0
        else:
            cmd.motor_cmd[i].kp = 200.0
            cmd.motor_cmd[i].kd = 5.0
        #print("Set Motors: Motor", i, "mode:", cmd.motor_cmd[i].mode)

    dt = 0.005

    while True:
        step_start = time.perf_counter()

        for i in range(NUM_MOTORS_H1):
           cmd.motor_cmd[i].q = default_joint_positions[i]
           cmd.motor_cmd[i].dq = 0.0
           cmd.motor_cmd[i].tau = 0.0


        #cmd.motor_cmd[i].position = default_joint_positions[i]

        # Compute CRC: Cycilic Redundancy Check
        cmd.crc = crc.Crc(cmd)
        # Publish
        pub.Write(cmd)

        # Sleep until next step
        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
