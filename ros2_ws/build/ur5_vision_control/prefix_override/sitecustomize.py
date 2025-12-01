import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/marco/Documents/ur5-vision-control/ros2_ws/install/ur5_vision_control'
