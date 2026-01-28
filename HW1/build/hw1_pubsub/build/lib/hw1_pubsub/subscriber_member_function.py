# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Hw1Subscriber(Node):

    def __init__(self):
        super().__init__('hw1_subscriber')
        self.subscription = self.create_subscription(
            String,
            'publish_integer',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        res = "even" if int(msg.data) % 2 == 0 else "odd"
        self.get_logger().info('I received %s. It is an %s number' % (msg.data, res))


def main(args=None):
    rclpy.init(args=args)

    hw1_subscriber = Hw1Subscriber()

    rclpy.spin(hw1_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hw1_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
