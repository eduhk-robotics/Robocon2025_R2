import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/eric/Robocon2025_R2/2025R2_ws/install/navigation'
