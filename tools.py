import math
import time
from dynamixel_sdk import *

class config:
    # Control table address for your DYNAMIXEL model
    ADDR_TORQUE_ENABLE = 64  # Address for Torque Enable
    ADDR_GOAL_POSITION = 116  # Address for Goal Position
    ADDR_PRESENT_POSITION = 132  # Address for Present Position

    # Protocol version
    PROTOCOL_VERSION = 2.0  # Protocol version used by your DYNAMIXEL mode

    # Default settings
    BAUDRATE = 1000000  # Default baudrate of DYNAMIXEL
    DEVICENAME = "/dev/ttyUSB0"  # Adjust the port name for your platform (e.g., 'COM3' on Windows)

    TORQUE_ENABLE = 1  # Value for enabling torque
    TORQUE_DISABLE = 0  # Value for disabling torque

    DXL_MOVING_STATUS_THRESHOLD = 50  # Dynamixel moving status threshold


def trans2para(degree):
    # 2048 -> 0 -> 180
    # 3072 -> Pi/2 -> 270
    # 1024 -> -Pi/2 -> 90
    r = 1024 * 2 / math.pi
    return int(2048 + r * degree)

def trans2degree(para):
    r = 1024 * 2 / math.pi
    degree = (para - 2048) / r
    return degree

class Motor:
    def __init__(self, id, min_th, max_th):
        self.id = id
        self.min_th = min_th
        self.max_th = max_th

    def set_torque(i: bool):
        if i:
            # set torque
            torque_code = packetHandler.write1ByteTxRx(
                portHandler, self.id, config.ADDR_TORQUE_ENABLE, config.TORQUE_ENABLE
            )[0]
            assert torque_code == COMM_SUCCESS
        else:
            torque_code = packetHandler.write1ByteTxRx(
                portHandler, self.id, config.ADDR_TORQUE_ENABLE, config.TORQUE_DISABLE
            )
            assert torque_code == COMM_SUCCESS

    def move_to_position(self, goal_degree):
        portHandler = PortHandler(config.DEVICENAME)
        packetHandler = PacketHandler(config.PROTOCOL_VERSION)

        # check if the port is open
        assert portHandler.openPort()
        assert portHandler.setBaudRate(config.BAUDRATE)

        # set torque
        self.set_torque(True)

        # check if the goal position is valid
        if goal_degree < self.min_th or goal_degree > self.max_th:
            packetHandler.write1ByteTxRx(
                portHandler, self.id, config.ADDR_TORQUE_ENABLE, config.TORQUE_DISABLE
            )
            print("Bad Goal Position")
            exit()

        # move to the goal position
        print(f"Moving to position {goal_degree}")
        while True:
            packetHandler.write4ByteTxRx(
                portHandler, self.id, config.ADDR_GOAL_POSITION, goal_degree
            )
            present_position, _, _ = packetHandler.read4ByteTxRx(
                portHandler, self.id, config.ADDR_PRESENT_POSITION
            )
            if abs(goal_degree - present_position) < config.DXL_MOVING_STATUS_THRESHOLD:
                break

        # packetHandler.write1ByteTxRx(portHandler, self.id, config.ADDR_TORQUE_ENABLE, config.TORQUE_DISABLE)
        present_position, _, _ = packetHandler.read4ByteTxRx(
            portHandler, self.id, config.ADDR_PRESENT_POSITION
        )
        portHandler.closePort()
        return present_position

    def get_position(self):
        portHandler = PortHandler(config.DEVICENAME)
        packetHandler = PacketHandler(config.PROTOCOL_VERSION)

        # check if the port is open
        assert portHandler.openPort()
        assert portHandler.setBaudRate(config.BAUDRATE)

        # get the present position
        present_position, _, _ = packetHandler.read4ByteTxRx(
            portHandler, self.id, config.ADDR_PRESENT_POSITION
        )
        portHandler.closePort()
        return trans2degree(present_position)

class OpenX:
    def __init__(self):
        # limit of the joint
        self.ll = [-3.141592653589793, -1.5, -1.5, -1.7, -0.277 * math.pi, -0.277 * math.pi]
        self.ul = [3.141592653589793, 1.5, 1.4, 1.97, 0.5 * math.pi, 0.5 * math.pi]
        
        self.m11 = Motor(11, trans2para(self.ll[0]) + 1, trans2para(self.ul[0]) + 1)
        self.m12 = Motor(12, trans2para(self.ll[1]) + 1, trans2para(self.ul[1]) + 1)
        self.m13 = Motor(13, trans2para(self.ll[2]) + 1, trans2para(self.ul[2]) + 1)
        self.m14 = Motor(14, trans2para(self.ll[3]) + 1, trans2para(self.ul[3]) + 1)
        self.m15 = Motor(15, trans2para(self.ll[4]) + 1, trans2para(self.ul[4]) + 1)
        
    def get_joint_real(self):
        s1 = self.m11.get_position()
        s2 = self.m12.get_position()
        s3 = self.m13.get_position()
        s4 = self.m14.get_position()
        s5 = self.m15.get_position()
        return [s1, s2, s3, s4, s5]
    
    def set_joint_real(self, angle_list):
        s1 = self.m11.move_to_position(angle_list[0])
        s3 = self.m12.move_to_position(angle_list[2])
        s2 = self.m13.move_to_position(angle_list[1])
        s4 = self.m14.move_to_position(angle_list[3])
        s5 = self.m15.move_to_position(angle_list[4])
        return s1, s2, s3, s4, s5
    
    def set_grasper_open_real(self, i: bool):

        if i:
            s5 = self.m15.move_to_position(trans2para(0.5 * math.pi))
        else:
            s5 = self.m15.move_to_position(trans2para(-0.276 * math.pi))
        return s5

if __name__ == "__main__":
    arm = OpenX()
    fi=arm.get_joint_real()
    ans = arm.set_joint_real([0, 0, 0, 0, 0])
    print(fi, ans)
