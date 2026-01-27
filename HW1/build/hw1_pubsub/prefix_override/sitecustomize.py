import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/xquan/ROE500-ROS/HW1/install/hw1_pubsub'
