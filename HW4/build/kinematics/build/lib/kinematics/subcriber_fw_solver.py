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

        # Calculate the forward kinematics and print the rotation and position of the end-effector
        T0_7 = self.fk_from_thetas(thetas)
        # Print the rotation and position of the end-effector
        self.print_rotation_position(T0_7)

    def print_rotation_position(self, T):
        """
        Print the rotation matrix and position vector components of a homogeneous transformation matrix.
        Args:
            T (numpy.ndarray): A 4x4 homogeneous transformation matrix containing rotation and position information.
        Returns:
            None
        """
        
        rotation = T[0:3, 0:3]
        position = T[0:3, 3]
        self.get_logger().info(f'\nRotation:\n{rotation}\nPosition:\n{position}')


    # Forward kinematics functions
    # Calculate the homogeneous transformation matrix from DH parameters
    def dh_standard(self, theta, d, a, alpha):
        """
        Computes the Denavit-Hartenberg (DH) standard transformation matrix.
        This method calculates the 4x4 homogeneous transformation matrix based on 
        the standard DH convention parameters. The transformation represents the 
        position and orientation of a joint frame relative to the previous frame.
        Args:
            theta (float): Joint angle (rotation about Z-axis) in radians.
            d (float): Link offset (translation along Z-axis).
            a (float): Link length (translation along X-axis).
            alpha (float): Link twist (rotation about X-axis) in radians.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix of the form:
                [[cos(θ), -sin(θ)cos(α), sin(θ)sin(α), a*cos(θ)],
                 [sin(θ), cos(θ)cos(α), -cos(θ)sin(α), a*sin(θ)],
                 [0, sin(α), cos(α), d],
                 [0, 0, 0, 1]]
        Note:
            Trigonometric values are rounded to 10 decimal places to avoid 
            floating-point precision errors.
        """

        ct, st = np.round(np.cos(theta), 10), np.round(np.sin(theta), 10)
        ca, sa = np.round(np.cos(alpha), 10), np.round(np.sin(alpha), 10)
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d],
            [0,  0,    0,  1]
        ])

    # Calculate the forward kinematics from a list of angles thetas
    def fk_from_thetas(self, thetas):
        """
        Calculate the forward kinematics of a 6-DOF robot arm using Denavit-Hartenberg parameters.
        This method computes the homogeneous transformation matrix from the base frame to the end-effector
        frame by multiplying individual transformation matrices for each joint based on standard DH convention.
        Args:
            thetas (list or array-like): A sequence of 6 joint angles in degrees corresponding to each
                                         revolute joint of the robot arm.
        Returns:
            np.ndarray: A 4x4 homogeneous transformation matrix representing the pose of the end-effector
                       relative to the base frame. The matrix combines rotation and position information.
        Raises:
            ValueError: If the length of thetas is not exactly 6.
        """
        

        if len(thetas) != 6:
            raise ValueError("Expected 6 joint angles, got {}".format(len(thetas)))
        
        # Cast thetas value to variable names for better readability
        theta1, theta2, theta3, theta4, theta5, theta6 = thetas

        #Physical mesurements of the robot
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

        # Calculate the forward kinematics using the DH parameters
        T = np.eye(4)
        for theta, d, a, alpha in DH:
            # Calculate the homogeneous transformation matrix for each joint and multiply them together
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