import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tristan/astra/ik_test/latest/ik_ros_ws/install/robot'
