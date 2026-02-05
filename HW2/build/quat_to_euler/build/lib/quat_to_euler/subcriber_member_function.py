import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray


class QuatToEulerSubscriber(Node):

    def __init__(self):
        super().__init__('quat_to_euler_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'quat_to_euler_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


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