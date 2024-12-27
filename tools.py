import time
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
        
    packetHandler.write1ByteTxRx(portHandler, motor.id, config.ADDR_TORQUE_ENABLE, config.TORQUE_DISABLE)
    present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, motor.id, config.ADDR_PRESENT_POSITION)
    portHandler.closePort()
    return present_position
    
    
if __name__ == "__main__":
    MOTOR_ID = 15
    DXL_MINIMUM_POSITION_VALUE = 1460 # Minimum position value
    DXL_MAXIMUM_POSITION_VALUE = 2700 # Maximum positio

    m = Motor(MOTOR_ID, DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE)
    s = move_to_position(m, 2600)
    print(s)