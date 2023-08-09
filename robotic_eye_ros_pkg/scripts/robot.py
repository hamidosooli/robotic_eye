#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time


"""  Class to organize the hardware writes for the 4 servos of the robot  """



if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *


MY_DXL = 'XL320'  # Servo name

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyACM0'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
TORQUE_MAX               = 1023 #Max torque

ADDR_TORQUE_ENABLE          = 24  #Torque enable address
ADDR_TORQUE_MAX    = 15            #Max torque
ADDR_GOAL_POSITION          = 30  #Goal Position Address
ADDR_PRESENT_POSITION       = 37 #Present Position Address
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the CW Angle Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 980     # Refer to the CCW Angle Limit of product eManual
BAUDRATE                    = 1000000   # Default Baudrate of XL-320 is 1Mbps

ADDR_RETURN_DELAY_TIME = 5 #Return Delay Time
DELAY_TIME = 0  #Delay Time

PROTOCOL_VERSION            = 2.0



#Servo IDs
DXL_ID_LEFT_X                      = 1
DXL_ID_LEFT_Y                     = 10
DXL_ID_RIGHT_X                      = 6
DXL_ID_RIGHT_Y                      = 5


# left_x, left_y, right_x, right_y
ids = [DXL_ID_LEFT_X, DXL_ID_LEFT_Y, DXL_ID_RIGHT_X, DXL_ID_RIGHT_Y]



#Robot class can be imported and used for all the hardware writes
class Robot:

    def __init__(self):

        #port and packet handler
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        #configures servos
        self.openPort()
        self.setBaudrate()
        self.enableTorque()
        self.setDelayTime()

    #Open port using port handler
    def openPort(self):
        try:
            self.portHandler.openPort()
            print("Succeeded to open the port")
        except:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

    #Sets delay time to all the servos
    def setDelayTime(self):
        for id in ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_RETURN_DELAY_TIME, DELAY_TIME)
            while dxl_comm_result != COMM_SUCCESS:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_RETURN_DELAY_TIME, DELAY_TIME)
                print("Error with ", id)
    
    #sets baud rate for all the servos
    def setBaudrate(self):
        try:
            self.portHandler.setBaudRate(BAUDRATE)
            print("Succeeded to change the baudrate")
        except:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    #enables torque on all the servos
    def enableTorque(self):
        for id in ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            while dxl_comm_result != COMM_SUCCESS:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
                print("Error with ", id)

            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_TORQUE_MAX, TORQUE_MAX)
            while dxl_comm_result != COMM_SUCCESS:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_TORQUE_MAX, TORQUE_MAX)
                print("Error with ", id)
            print("DYNAMIXEL has been successfully connected")
            
            

        print("Ready to get & set Position.")

    
    #sets positions to all the servos
    def setPositions(self, positions):

        # X Direction servo hardware writes

        #Makes sure that both target positions are in range for both x servos
        #if one servo target position is out of range it moves neither
        if(inRange(positions[0])and inRange(positions[2])):

            #Writes both servos to go to target position
            while 1:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ids[0], ADDR_GOAL_POSITION, clip(positions[0]))
                if dxl_comm_result == COMM_SUCCESS:
                    break

            while 1:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ids[2], ADDR_GOAL_POSITION, clip(positions[2]))
                if dxl_comm_result == COMM_SUCCESS:
                    break


        # Y Direction servo hardware writes

        #Makes sure that both target positions are in range for both x servos
        #if one servo target position is out of range it moves neither
        if(inRange(positions[1])and inRange(positions[3])):
            #Writes both servos to go to target position
            while 1:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ids[1], ADDR_GOAL_POSITION, clip(positions[1]))
                if dxl_comm_result == COMM_SUCCESS:
                    break

            while 1:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ids[3], ADDR_GOAL_POSITION, clip(positions[3]))
                if dxl_comm_result == COMM_SUCCESS:
                    break

    #method to increment the servo positions by a certain amount relative to their current position
    def incrementPositions(self, positions):
        pos = []

        #reads servo positions
        currentPos = self.getPositions()

        #converts incremental values into actual servo positions
        for i in range(4):
            pos.append(positions[i] + currentPos[i])

        self.setPositions(pos)

    #Reads all 4 servo positions
    def getPositions(self):
        positions = []

        for i in range(4):
            while 1:
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ids[i], ADDR_PRESENT_POSITION)
                if dxl_comm_result == COMM_SUCCESS:
                    break

            positions.append(dxl_present_position)

        
        return positions



#method to clip servo target positions to servo range
def clip(position):
    if position>DXL_MAXIMUM_POSITION_VALUE:
        return DXL_MAXIMUM_POSITION_VALUE
    elif position<DXL_MINIMUM_POSITION_VALUE:
        return DXL_MINIMUM_POSITION_VALUE
    else:
        return round(position)

#boolean method to check if a target position is in range
def inRange(position):
    return DXL_MINIMUM_POSITION_VALUE<= position and position<=DXL_MAXIMUM_POSITION_VALUE