#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt


class COMPlotter(Node):

    def __init__(self):
        super().__init__('com_plotter')

        urdf_path = "/home/hitech/work/code/g1_robot_ws/src/g1_description/urdf/g1_29dof_mode_16.urdf"

        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()

        self.q = pin.neutral(self.model)

        self.com_history = []

        self.sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.plot_com)

        self.get_logger().info("COM plotter started")

    def joint_callback(self, msg):

        for i, name in enumerate(msg.name):

            if name in self.model.names:

                idx = self.model.getJointId(name)

                if idx > 0:
                    self.q[idx-1] = msg.position[i]

        pin.centerOfMass(self.model, self.data, self.q)

        com = self.data.com[0]

        self.com_history.append(com.copy())

        self.get_logger().info(
            f"COM: x={com[0]:.3f}, y={com[1]:.3f}, z={com[2]:.3f}",
            throttle_duration_sec=1.0
        )

    def plot_com(self):

        if len(self.com_history) < 10:
            return

        data = np.array(self.com_history)

        plt.clf()

        plt.subplot(3,1,1)
        plt.plot(data[:,0])
        plt.title("CoM X")

        plt.subplot(3,1,2)
        plt.plot(data[:,1])
        plt.title("CoM Y")

        plt.subplot(3,1,3)
        plt.plot(data[:,2])
        plt.title("CoM Z")

        plt.pause(0.01)


def main():

    rclpy.init()

    node = COMPlotter()

    plt.ion()

    rclpy.spin(node)


if __name__ == "__main__":
    main()
