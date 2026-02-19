import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class FwSolverSubscriber(Node):

    def __init__(self):
        super().__init__('fw_solver_subscriber')
        self.topic_name = 'fw_solver_topic'

        self.subscription = self.create_subscription(
            Float32MultiArray,
            self.topic_name,
            self.listener_callback,
            10)
        
        self.get_logger().info(
            f"fw_solver_subscriber node started."
        )
        self.get_logger().info(
            f"Listening on topic: {self.topic_name} \r"
            "(Float32MultiArray: [q1, q2, q3, q4, q5, q6])"
        )

        self.subscription  # prevent unused variable warning
        

    def listener_callback(self, msg):        

        thetas = msg.data
        
        self.get_logger().info(f'\nReceived joint angles: "{thetas}"')

        T0_7 = self.fk_from_thetas(thetas)
        self.print_rotation_position(T0_7)

    def print_rotation_position(self, T):
        rotation = T[0:3, 0:3]
        position = T[0:3, 3]
        self.get_logger().info(f'\nRotation:\n{rotation}\nPosition:\n{position}')

    def dh_standard(self, theta, d, a, alpha):
        ct, st = np.round(np.cos(theta), 10), np.round(np.sin(theta), 10)
        ca, sa = np.round(np.cos(alpha), 10), np.round(np.sin(alpha), 10)
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d],
            [0,  0,    0,  1]
        ])

    def fk_from_thetas(self, thetas):
        if len(thetas) != 6:
            raise ValueError("Expected 6 joint angles, got {}".format(len(thetas)))
        
        theta1, theta2, theta3, theta4, theta5, theta6 = thetas
        d1, d5, d6 = 475, 720, 85
        a1, a2, a4 = 150, 600, 120

        DH = [
            (theta1, d1, a1, 90),
            (theta2 + 90, 0, a2, 0),
            (theta3, 0, a4, 90),
            (theta4 + 90, d5, 0, -90),
            (theta5, 0, 0, 90),
            (theta6, d6, 0, 0)
        ]

        T = np.eye(4)
        for theta, d, a, alpha in DH:
            dh_std = self.dh_standard(np.deg2rad(theta), d, a, np.deg2rad(alpha))
            T = T @ dh_std
        return T


def main(args=None):
    rclpy.init(args=args)

    fw_solver_subscriber = FwSolverSubscriber()

    rclpy.spin(fw_solver_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fw_solver_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()