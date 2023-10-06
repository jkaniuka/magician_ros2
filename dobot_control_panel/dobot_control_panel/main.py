#!/usr/bin/env python3

import sys
from rqt_gui.main import Main

def main():
    main = Main()
    sys.exit(main.main(sys.argv, standalone='dobot_control_panel.ros2_dobot_control_panel.Ros2DobotControlPanel'))

if __name__ == '__main__':
    main()