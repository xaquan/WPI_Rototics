import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from .ConverterHelper import ConverterHelper
from .KinematicModel import KinematicModel


class InvSolverSubscriber(Node):

    def __init__(self):
        super().__init__('inv_solver_subscriber')
        self.topic_name = 'inv_solver_topic'
        self.subscription = self.create_subscription(
            Pose,
            self.topic_name,
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.get_logger().info(
            f"inv_solver_subscriber node started."
        )
        self.get_logger().info(
            f"Listening on topic: {self.topic_name} \r"
            "(geometry_msgs.msg.pose: {position: {x, y, z}, orientation: {w, x, y, z}})"
        )

    def listener_callback(self, msg):
        self.get_logger().info('Received a pose')
        self.get_logger().info(f'Position: "{msg.position.x}, {msg.position.y}, {msg.position.z}"')
        self.get_logger().info(f'Orientation: "{msg.orientation.x}, {msg.orientation.y}, {msg.orientation.z}, {msg.orientation.w}"')

        self.get_logger().info('Calculating inverse kinematics...')
        # Calculate the inverse kinematics and print the joint angles
        kModel = KinematicModel()
        theta = kModel.inverse_kinematics_ABB_1400(msg)

        # rad to deg
        theta = np.degrees(theta)

        # Print the joint angles
        self.get_logger().info(f'Joint Angles: {theta}')

def main(args=None):
    rclpy.init(args=args)

    inv_solver_subscriber = InvSolverSubscriber()

    rclpy.spin(inv_solver_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    inv_solver_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()