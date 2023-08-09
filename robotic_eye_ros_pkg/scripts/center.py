#!/usr/bin/env python3

from robot import Robot


#small program to set all servos to center
#execute as a python file
def main():
    robot = Robot()

    robot.setPositions([512, 512, 512, 512])


if __name__ == '__main__':
    main()