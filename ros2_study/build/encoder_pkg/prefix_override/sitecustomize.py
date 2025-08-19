import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/min/my-ros2-project/ros2_study/install/encoder_pkg'
