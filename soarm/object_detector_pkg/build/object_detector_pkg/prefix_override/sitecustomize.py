import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/root/soarm/object_detector_pkg/install/object_detector_pkg'
