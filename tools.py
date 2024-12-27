import time
import math
from dynamixel_sdk import *

class config:
    # Control table address for your DYNAMIXEL model
    ADDR_TORQUE_ENABLE = 64          # Address for Torque Enable
    ADDR_GOAL_POSITION = 116         # Address for Goal Position
    ADDR_PRESENT_POSITION = 132      # Address for Present Position
    
    # Protocol version
    PROTOCOL_VERSION = 2.0           # Protocol version used by your DYNAMIXEL mode
    
    # Default settings
    BAUDRATE = 1000000                 # Default baudrate of DYNAMIXEL
    DEVICENAME = '/dev/ttyUSB0'      # Adjust the port name for your platform (e.g., 'COM3' on Windows)
        
    TORQUE_ENABLE = 1                # Value for enabling torque
    TORQUE_DISABLE = 0               # Value for disabling torque
    
    DXL_MOVING_STATUS_THRESHOLD = 50 # Dynamixel moving status threshold
    
def trans2para(degree):
    # 2048 -> 0 -> 180
    # 3072 -> Pi/2 -> 270
    # 1024 -> -Pi/2 -> 90
    r = 1024*2/math.pi
    return int(2048 + r * degree)
    
class Motor:
    def __init__(self, id, min_th, max_th):
        self.id = id
        self.min_th = min_th
        self.max_th = max_th

def move_to_position(motor: Motor, goal_degree):
    portHandler = PortHandler(config.DEVICENAME)
    packetHandler = PacketHandler(config.PROTOCOL_VERSION)
    
    assert(portHandler.openPort())
       
    assert(portHandler.setBaudRate(config.BAUDRATE))
        
    torque_code = packetHandler.write1ByteTxRx(portHandler, motor.id, config.ADDR_TORQUE_ENABLE, config.TORQUE_ENABLE)[0]

    assert(torque_code == COMM_SUCCESS)
        
    if goal_degree < motor.min_th or goal_degree > motor.max_th:
        packetHandler.write1ByteTxRx(portHandler, motor.id, config.ADDR_TORQUE_ENABLE, config.TORQUE_DISABLE)
        print("Bad Goal Position")
        exit()
        
    print(f"Moving to position {goal_degree}")
    while(True):
        packetHandler.write4ByteTxRx(portHandler, motor.id, config.ADDR_GOAL_POSITION, goal_degree)
        present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, motor.id, config.ADDR_PRESENT_POSITION)

        if abs(goal_degree - present_position) < config.DXL_MOVING_STATUS_THRESHOLD:
            break
        
    # packetHandler.write1ByteTxRx(portHandler, motor.id, config.ADDR_TORQUE_ENABLE, config.TORQUE_DISABLE)
    present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, motor.id, config.ADDR_PRESENT_POSITION)
    portHandler.closePort()
    return present_position
    

def set_joint_real(angle_list):
    
    # ll = [-3.141592653589793, -1.5, -1.5, -1.7, -0.01, -0.01] 
    # ul = [3.141592653589793, 1.5, 1.4, 1.97, 0.019, 0.019]
    
    ll = [-3.141592653589793, -1.5, -1.5, -1.7, -0.01, -0.01] 
    ul = [3.141592653589793, 1.5, 1.4, 1.97, 0.019, 0.019]
    
    m11 = Motor(11, trans2para(ll[0])+1, trans2para(ul[0])+1)
    m12 = Motor(12, trans2para(ll[1])+1, trans2para(ul[1])+1)
    m13 = Motor(13, trans2para(ll[2])+1, trans2para(ul[2])+1)
    m14 = Motor(14, trans2para(ll[3])+1, trans2para(ul[3])+1)
    m15 = Motor(15, trans2para(ll[4])+1, trans2para(ul[4])+1)

    angle_list = [trans2para(angle) for angle in angle_list]
    
    s1 = move_to_position(m11, angle_list[0])
    s3 = move_to_position(m13, angle_list[2])
    s2 = move_to_position(m12, angle_list[1])
    s4 = move_to_position(m14, angle_list[3])
    s5 = move_to_position(m15, angle_list[4])
    
    return s1, s2, s3, s4, s5

def set_grasper_open_real(i: bool):
    
    ll = [-3.141592653589793, -1.5, -1.5, -1.7, -0.277*math.pi, -0.277*math.pi]
    ul = [3.141592653589793, 1.5, 1.4, 1.97, 0.5*math.pi, 0.5*math.pi]
    
    m15 = Motor(15, trans2para(ll[4])+1, trans2para(ul[4])+1)
    if i:
        s5 = move_to_position(m15, trans2para(0.5*math.pi))
    else:
        s5 = move_to_position(m15, trans2para(-0.276*math.pi))
    return s5

if __name__ == "__main__":
    # MOTOR_ID = 15
    # DXL_MINIMUM_POSITION_VALUE = 0 # Minimum position value
    # DXL_MAXIMUM_POSITION_VALUE = 8700 # Maximum positio

    # m15 = Motor(15, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE)
    # m14 = Motor(14, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE)
    # m13 = Motor(13, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE)
    # m12 = Motor(12, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE)
    # m11 = Motor(11, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE)
    
    # s = move_to_position(m11, 2048)
    # s = move_to_position(m12, 2048)
    # s = move_to_position(m13, 2048)
    # s = move_to_position(m14, 2048)
    # s = move_to_position(m15, 2048)
    
    #ans = set_joint_real((2.3751978890626555, 0.6717358345059067, 0.021551643358042705, 0.008259045441259468, 0.0))
    # ans = set_joint_real((0.7424305449546253, -0.8227422996081575, 0.49246695964874115, 1.2211528463550654, 0.0))
    ans = set_joint_real([0, 0, 0, 0, 0])
    
    print(ans)
