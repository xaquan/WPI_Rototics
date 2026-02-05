import rclpy
from rclpy.node import Node
import math

from std_msgs.msg import Float32MultiArray

class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

class EulerAngles:
    def __init__(self, roll, pitch, yaw):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

class QuatToEulerSubscriber(Node):

    def __init__(self):
        super().__init__('quat_to_euler_subscriber')
        self.topic_name = 'quat_to_euler_topic'
        self.subscription = self.create_subscription(
            Float32MultiArray,
            self.topic_name,
            self.listener_callback,
            10)

        self.get_logger().info(
            f"quat_to_euler_node started."
        )
        self.get_logger().info(
            f"Listening on topic: {self.topic_name} "
            "(Float32MultiArray: [x, y, z, w])"
        )

        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        quat = Quaternion(x=msg.data[0], y=msg.data[1], z=msg.data[2], w=msg.data[3])
        euler_angles = self.convert_quat_to_euler(quat)

        self.get_logger().info('Euler angles (radians): roll=%.4f, pitch=%.4f, yaw=%.4f' %
            (euler_angles['roll'], euler_angles['pitch'], euler_angles['yaw']))



    def convert_quat_to_euler(self, quat):
        """
        Convert quaternion to Euler angles in 3-2-1 sequence (roll, pitch, yaw).
        
        Args:
            quat: dict or object with keys/attributes w, x, y, z (normalized quaternion)
            
        Returns:
            dict with keys 'roll', 'pitch', 'yaw' (in radians)
        """
        # Extract quaternion components
        w = quat.w
        x = quat.x
        y = quat.y
        z = quat.z
        
        # roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # pitch (y-axis rotation)
        sinp = math.sqrt(1 + 2 * (w * y - x * z))
        cosp = math.sqrt(1 - 2 * (w * y - x * z))
        pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2
        
        # yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return {'roll': roll, 'pitch': pitch, 'yaw': yaw}


def main(args=None):
    rclpy.init(args=args)

    quat_to_euler_subscriber = QuatToEulerSubscriber()

    rclpy.spin(quat_to_euler_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    quat_to_euler_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()