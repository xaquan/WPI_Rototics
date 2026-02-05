import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/code/WPI_Rototics/HW2/install/quat_to_euler'
