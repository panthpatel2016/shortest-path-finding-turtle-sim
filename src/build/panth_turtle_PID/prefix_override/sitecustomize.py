import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/panthpatel_/ros2_ws2/src/install/panth_turtle_PID'
