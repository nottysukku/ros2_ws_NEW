import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sukritchopra/ros2_ws/src/jetrover_description/install/jetrover_description'
