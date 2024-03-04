#!/usr/bin/env python

import os
import time
from dynamixel_sdk import *                 # Uses Dynamixel SDK library

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




class USB_DEVICE:
    def __init__(self):
        pass
        
    def usb_initialization(self, usb='/dev/ttyUSB0', baudrate=1000000, protocol_version=2.0):
        self.usb = usb
        self.baudrate = baudrate
        self.protocol_version = protocol_version
        self.portHandler = PortHandler(self.usb)
        self.packetHandler = PacketHandler(self.protocol_version)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

    def set_baudrate(self):
        # Set port baudrate
        if self.portHandler.setBaudRate(self.baudrate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    def close(self):
        self.portHandler.closePort()




class MOTOR_2_WHEEL_MODE(USB_DEVICE):
    def __init__(self):
        pass

    def motor_initialization(self, m1_id=1, m2_id=2):
        self.m1_id = m1_id
        self.m2_id = m2_id

        DXL_MINIMUM_POSITION_VALUE  = 0   # Refer to the Minimum Position Limit of product eManual
        DXL_MAXIMUM_POSITION_VALUE  = 4095    # Refer to the Maximum Position Limit of product eManual
        dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

        # motor initialization
        m1_comm_result, m1_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.m1_id, 64, dxl_goal_position[0])
        m2_comm_result, m2_error = self.packetHandler.write4ByteTxRx(self.portHandler, self.m2_id, 64, dxl_goal_position[0])

        # Set mode
        self.packetHandler.write1ByteTxRx(self.portHandler,self.m1_id, 11, 1)
        self.packetHandler.write1ByteTxRx(self.portHandler,self.m2_id, 11, 1)

        # Torque Enable
        self.packetHandler.write1ByteTxRx(self.portHandler,self.m1_id, 64, 1)
        self.packetHandler.write1ByteTxRx(self.portHandler,self.m2_id, 64, 1)


    def ping(self):
        m1_model_number, m1_comm_result, m1_error = self.packetHandler.ping(self.portHandler, self.m1_id)
        if m1_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(m1_comm_result))
        elif m1_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(m1_error))
        else:
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.m1_id, m1_model_number))

        m2_model_number, m2_comm_result, m2_error = self.packetHandler.ping(self.portHandler, self.m2_id)
        if m2_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(m2_comm_result))
        elif m2_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(m2_error))
        else:
            print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.m2_id, m2_model_number))


    def setSpeed(self, m1_speed, m2_speed):
        # Motor 1
        self.packetHandler.write4ByteTxRx(self.portHandler,self.m1_id, 104, int(m1_speed))    # Goal Velocity
        # self.packetHandler.write2ByteTxRx(self.portHandler,self.m1_id, 76, 1920)    # Velocity I Gain
        # self.packetHandler.write2ByteTxRx(self.portHandler,self.m1_id, 78, 100)     # 	Velocity P Gain

        # Motor 2
        self.packetHandler.write4ByteTxRx(self.portHandler,self.m2_id, 104, int(m2_speed))
        # self.packetHandler.write2ByteTxRx(self.portHandler,self.m2_id, 76, 1920)
        # self.packetHandler.write2ByteTxRx(self.portHandler,self.m2_id, 78, 100)



def init_motor() -> MOTOR_2_WHEEL_MODE:
    motor = MOTOR_2_WHEEL_MODE()
    motor.usb_initialization(usb='/dev/ttyUSB0', baudrate=1000000, protocol_version=2.0)
    motor.motor_initialization(m1_id=1, m2_id=2)
    motor.ping()
    motor.setSpeed(0, 0)
    return motor



if __name__ == '__main__':
    motor = MOTOR_2_WHEEL_MODE()
    motor.usb_initialization(usb='/dev/ttyUSB0', baudrate=1000000, protocol_version=2.0)
    motor.motor_initialization(m1_id=1, m2_id=2)
    motor.ping()
    motor.setSpeed(100, 100)
    time.sleep(1)
    motor.setSpeed(-200, -200)
    time.sleep(1)
    motor.setSpeed(0, 0)


