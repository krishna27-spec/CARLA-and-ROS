import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/srikrishna/motion_planning_project/ros2_ws/install/controls_pkg'
