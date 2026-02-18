import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/javierbecerril/harmonic_ws/install/tag_hover_two_tags'
