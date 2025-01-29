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

stand_down_joint_pos = np.array([
    0.0,  # 0  RightHipRoll
    0.0,  # 1  RightHipPitch
    0.5,  # 2  RightKnee
    0.0,  # 3  LeftHipRoll
    0.0,  # 4  LeftHipPitch
    0.5,  # 5  LeftKnee
    0.0,  # 6  WaistYaw
    0.0,  # 7  LeftHipYaw
    0.0,  # 8  RightHipYaw
    0.0,  # 9  NotUsed
    0.0,  # 10 LeftAnkle
    0.0,  # 11 RightAnkle
    0.0,  # 12 RightShoulderPitch
    0.0,  # 13 RightShoulderRoll
    0.0,  # 14 RightShoulderYaw
    0.0,  # 15 RightElbow
    0.0,  # 16 LeftShoulderPitch
    0.0,  # 17 LeftShoulderRoll
    0.0,  # 18 LeftShoulderYaw
    0.0,  # 19 LeftElbow
], dtype=float)

# Example posture angles for a "stand up" posture.
# Replace these with real stand angles for H1!
stand_up_joint_pos = np.array([
    0.0,   # 0  RightHipRoll
    -0.7,  # 1  RightHipPitch
    1.4,   # 2  RightKnee
    0.0,   # 3  LeftHipRoll
    -0.7,  # 4  LeftHipPitch
    1.4,   # 5  LeftKnee
    0.0,   # 6  WaistYaw
    0.0,   # 7  LeftHipYaw
    0.0,   # 8  RightHipYaw
    0.0,   # 9  NotUsed
    0.0,   # 10 LeftAnkle
    0.0,   # 11 RightAnkle
    0.0,   # 12 RightShoulderPitch
    0.0,   # 13 RightShoulderRoll
    0.0,   # 14 RightShoulderYaw
    0.0,   # 15 RightElbow
    0.0,   # 16 LeftShoulderPitch
    0.0,   # 17 LeftShoulderRoll
    0.0,   # 18 LeftShoulderYaw
    0.0,   # 19 LeftElbow
], dtype=float)


"""start_pos = np.zeros(NUM_MOTORS_H1, dtype=float)
start_pos[H1JointIndex.kRightHipPitch]  = -0.6
start_pos[H1JointIndex.kRightKnee]      =  1.2
start_pos[H1JointIndex.kLeftHipPitch]   = -0.6
start_pos[H1JointIndex.kLeftKnee]       =  1.2

start_pos[H1JointIndex.kRightShoulderPitch] = -0.6
start_pos[H1JointIndex.kLeftShoulderPitch]  = -0.6

end_pos = np.zeros(NUM_MOTORS_H1, dtype=float)
end_pos[H1JointIndex.kRightHipPitch]  = -0.6
end_pos[H1JointIndex.kRightKnee]      =  1.2
end_pos[H1JointIndex.kLeftHipPitch]   = -0.6
end_pos[H1JointIndex.kLeftKnee]       =  1.2

end_pos[H1JointIndex.kRightElbow] = 0.5
end_pos[H1JointIndex.kLeftElbow]  = -0.5"""

stand_pos = np.zeros(NUM_MOTORS_H1, dtype=float)

# Legs slightly bent for balance
stand_pos[H1JointIndex.kRightHipPitch]  = -0.1
stand_pos[H1JointIndex.kRightKnee]      =  0.0
stand_pos[H1JointIndex.kLeftHipPitch]   = -0.1
stand_pos[H1JointIndex.kLeftKnee]       =  0.0

# Fix ankles parallel to the ground
stand_pos[H1JointIndex.kRightAnkle] = 0.0
stand_pos[H1JointIndex.kLeftAnkle]  = 0.0

# Arms bent slightly in a relaxed position
stand_pos[H1JointIndex.kRightShoulderPitch] = 0.4
stand_pos[H1JointIndex.kLeftShoulderPitch]  = 0.4
stand_pos[H1JointIndex.kRightElbow] = 0.5
stand_pos[H1JointIndex.kLeftElbow]  = -0.5

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
        cmd.motor_cmd[i].kp = 80.0
        cmd.motor_cmd[i].kd = 5.0
        #print("Set Motors: Motor", i, "mode:", cmd.motor_cmd[i].mode)

    dt = 0.005
    sim_time = 0.0
    transition_duration = 0.00001

    while True:
        step_start = time.perf_counter()
        sim_time += dt

        #if sim_time < stand_transition_duration:
            # Interpolate from "down" to "up"
        #alpha = sim_time / stand_transition_duration
        alpha = min(sim_time / transition_duration, 1.0)

        for i in range(NUM_MOTORS_H1):
            desired_q = alpha * stand_up_joint_pos[i] + (1 - alpha)*stand_down_joint_pos[i]
            #cmd.motor_cmd[i].q = desired_q
            cmd.motor_cmd[i].q = stand_pos[i]
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].tau = 0.0
        #else:
            # Once we are "stood up," just hold the final posture
            #for i in range(NUM_MOTORS_H1):
                #cmd.motor_cmd[i].q = stand_up_joint_pos[i]
                #cmd.motor_cmd[i].dq = 0.0
                #cmd.motor_cmd[i].tau = 0.0

        # Compute CRC
        cmd.crc = crc.Crc(cmd)
        # Publish
        pub.Write(cmd)

        # Sleep until next step
        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
