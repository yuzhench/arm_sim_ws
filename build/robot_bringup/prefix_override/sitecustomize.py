import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yuzhench/Desktop/Intership/smart_arm/arm_sim_ws/install/robot_bringup'
