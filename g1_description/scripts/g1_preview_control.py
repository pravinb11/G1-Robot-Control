#!/usr/bin/env python3
"""
G1 Standing Controller
Implements a PD controller to hold the robot in a standing pose.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

LOCK_MODE = True
TEST_JOINT = "right_knee_joint"

# Only joints that are REVOLUTE and in controllers.yaml
STANDING_POSE = {
    'left_hip_pitch_joint':    -2.2, #-0.2,
    'left_hip_roll_joint':      0.0,
    'left_hip_yaw_joint':       0.0,
    'left_knee_joint':          2.2, #0.2, 
    'left_ankle_pitch_joint':  -0.2, #-0.2,
    'left_ankle_roll_joint':    0.0,
    'right_hip_pitch_joint':   2.2, #-0.2,
    'right_hip_roll_joint':     0.0,
    'right_hip_yaw_joint':      0.0,
    'right_knee_joint':         0.0,
    'right_ankle_pitch_joint': -0.2, #-0.2,
    'right_ankle_roll_joint':   0.0,
    'waist_yaw_joint':          0.0,
    # waist_roll_joint and waist_pitch_joint are FIXED joints - excluded
}

# PD_GAINS = {
#     'left_hip_pitch_joint':    {'kp': 50.0, 'kd': 20.0},
#     'left_hip_roll_joint':     {'kp': 50.0, 'kd': 20.0},
#     'left_hip_yaw_joint':      {'kp': 30.0, 'kd': 12.0},
#     'left_knee_joint':         {'kp': 70.0, 'kd': 25.0},
#     'left_ankle_pitch_joint':  {'kp': 20.0, 'kd': 8.0},
#     'left_ankle_roll_joint':   {'kp': 20.0, 'kd': 8.0},
#     'right_hip_pitch_joint':   {'kp': 50.0, 'kd': 20.0},
#     'right_hip_roll_joint':    {'kp': 50.0, 'kd': 20.0},
#     'right_hip_yaw_joint':     {'kp': 30.0, 'kd': 12.0},
#     'right_knee_joint':        {'kp': 70.0, 'kd': 25.0},
#     'right_ankle_pitch_joint': {'kp': 20.0, 'kd': 8.0},
#     'right_ankle_roll_joint':  {'kp': 20.0, 'kd': 8.0},
#     'waist_yaw_joint':         {'kp': 30.0, 'kd': 12.0},
# }

PD_GAINS = {
    'left_hip_pitch_joint':    {'kp': 10.0,  'kd': 5.0},
    'left_hip_roll_joint':     {'kp': 10.0,  'kd': 2.0},
    'left_hip_yaw_joint':      {'kp': 10.0,  'kd': 2.0},
    'left_knee_joint':         {'kp': 3.0,  'kd': 1.0},
    'left_ankle_pitch_joint':  {'kp': 10.0,  'kd': 0.0},
    'left_ankle_roll_joint':   {'kp': 10.0,  'kd': 0.0},
    'right_hip_pitch_joint':   {'kp': 10.0,  'kd': 5.0},
    'right_hip_roll_joint':    {'kp': 10.0,  'kd': 2.0},
    'right_hip_yaw_joint':     {'kp': 10.0,  'kd': 2.0},
    'right_knee_joint':        {'kp': 3.0,  'kd': 1.0},
    'right_ankle_pitch_joint': {'kp': 10.0,  'kd': 0.0},
    'right_ankle_roll_joint':  {'kp': 10.0,  'kd': 0.0},
    'waist_yaw_joint':         {'kp': 10.0,  'kd': 0.0},
}

EFFORT_LIMITS = {
    'left_hip_pitch_joint':    139.0,
    'left_hip_roll_joint':     139.0,
    'left_hip_yaw_joint':       88.0,
    'left_knee_joint':         139.0,
    'left_ankle_pitch_joint':   35.0,
    'left_ankle_roll_joint':    35.0,
    'right_hip_pitch_joint':   139.0,
    'right_hip_roll_joint':    139.0,
    'right_hip_yaw_joint':      88.0,
    'right_knee_joint':        139.0,
    'right_ankle_pitch_joint':  35.0,
    'right_ankle_roll_joint':   35.0,
    'waist_yaw_joint':          88.0,
}

JOINT_ORDER = list(STANDING_POSE.keys())


class StandingController(Node):

    def __init__(self):
        super().__init__('g1_standing_controller')

        self.joint_positions = {}
        self.joint_velocities = {}
        self.joints_received_logged = False

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        # Replace the subscription with this
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            qos
        )

        self.effort_pub = self.create_publisher(
            Float64MultiArray,
            '/effort_controller/commands',
            10
        )

        self.timer = self.create_timer(0.002, self.control_loop)
        self.get_logger().info('G1 Standing Controller started')
        self.get_logger().info(f'Controlling {len(JOINT_ORDER)} joints')

    def joint_state_callback(self, msg: JointState):
        if not self.joints_received_logged:
            self.get_logger().info(f'Velocity length: {len(msg.velocity)}') 
        for i, name in enumerate(msg.name):
            if name in STANDING_POSE:
                self.joint_positions[name] = msg.position[i]
                if len(msg.velocity) > i:
                    self.joint_velocities[name] = msg.velocity[i]
        

    def control_loop(self):
        received = len(self.joint_positions)
        total = len(JOINT_ORDER)

        if received < total:
            self.get_logger().info(
                f'Waiting for joint states: {received}/{total}',
                throttle_duration_sec=2.0
            )
            return
        
        # Print once when we first receive all joints
        if not self.joints_received_logged:
            self.get_logger().info(f'Received all {total} joints — starting control loop')
            self.joints_received_logged = True

        efforts = []
        for joint in JOINT_ORDER:
            q    = self.joint_positions.get(joint, 0.0)
            dq   = self.joint_velocities.get(joint, 0.0)

            q_des = STANDING_POSE[joint]
            kp   = PD_GAINS[joint]['kp']
            kd   = PD_GAINS[joint]['kd']

            # if LOCK_MODE and joint != TEST_JOINT:
            #     # q_des = 0   # hold current position
            #     q_des = STANDING_POSE[joint]
            #     kp = PD_GAINS[joint]['kp']
            #     kd = PD_GAINS[joint]['kd']
            #     # kp = 150.0
            #     # kd = 40.0
            # else:
            #     q_des = STANDING_POSE[joint]
            #     kp = PD_GAINS[joint]['kp']
            #     kd = PD_GAINS[joint]['kd']
        
            tau = kp * (q_des - q) - kd * dq
            tau = float(np.clip(tau, -EFFORT_LIMITS[joint], EFFORT_LIMITS[joint]))
            efforts.append(tau)

        msg = Float64MultiArray()
        msg.data = efforts
        self.effort_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StandingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()