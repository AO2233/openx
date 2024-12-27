import pybullet as p
import numpy as np
import time

DEBUG = True
# DEBUG = False

def model_info(model_id):
    num_joints = p.getNumJoints(model_id)
    for joint_index in range(num_joints):
        joint_info = p.getJointInfo(model_id, joint_index)
        link_state = p.getLinkState(model_id, joint_index)
        
        print(f"关节索引: {joint_index}")
        print(f"  关节名称: {joint_info[1].decode('utf-8')}")
        print(f"  关节类型: {joint_info[2]}")
        print(f"  位置索引: {joint_info[3]}")
        print(f"  速度索引: {joint_info[4]}")
        print(f"  标志位: {joint_info[5]}")
        print(f"  阻尼系数: {joint_info[6]}")
        print(f"  摩擦系数: {joint_info[7]}")
        print(f"  最小位置限制: {joint_info[8]}")
        print(f"  最大位置限制: {joint_info[9]}")
        print(f"  最大施加力: {joint_info[10]}")
        print(f"  最大速度: {joint_info[11]}")
        print(f"  链接名称: {joint_info[12].decode('utf-8')}")
        print(f"  关节的旋转轴: {joint_info[13]}")
        print(f"  父框架位置: {joint_info[14]}")
        print(f"  父框架方向（四元数）: {joint_info[15]}")
        print(f"  父链接索引: {joint_info[16]}")
        print(f"  关节位置: {link_state[0]}")       
        print(f"  关节方向: {p.getEulerFromQuaternion(link_state[1])}")       
        print("-" * 40)

def set_angle(model_id, angle_list) -> list:
    # index [1,2,3,4,5,6]
    for i in range(1,7):
        p.resetJointState(model_id,i,angle_list[i-1])
    return angle_list

def get_angle(model_id) -> list:
    angle_list = []
    for i in range(1,7): # index [1,2,3,4,5,6]
        angle_list.append(p.getJointState(model_id,i)[0])
    return angle_list
    
def move_to_point_line(model_id, target_position, target_orientation=None, num_steps=100):
    if target_orientation is not None:
        print("target_orientation is not None, it's dengerous to use move_to_point_line.")
    
    ll = [-3.141592653589793, -1.5, -1.5, -1.7, -0.01, -0.01] 
    ul = [3.141592653589793, 1.5, 1.4, 1.97, 0.019, 0.019]
    jr = [ul[i] - ll[i] for i in range(len(ll))]
    rs = [0, 0, 0, 0, 0, 0]
    
    threshold = 1e-5
    max_iterations = int(1e4) # cpp-api need to ensure the type is int
    end_effector_index = 8
    
    assert target_position is not None
    now_position = p.getLinkState(model_id, end_effector_index)[0]
    trajectory = np.linspace(now_position, target_position, num_steps)
    
    if DEBUG:
         for i in range(len(trajectory) - 1):
            p.addUserDebugLine(
                trajectory[i], trajectory[i + 1], [1, 0, 0], lineWidth=2
            )
    
    for target_point in trajectory:
        # 计算逆运动学，得到关节角度
        joint_angles = p.calculateInverseKinematics(
            model_id,
            end_effector_index,
            target_point,
            targetOrientation=target_orientation,
            residualThreshold=threshold,
            maxNumIterations=max_iterations,
            lowerLimits=ll,
            upperLimits=ul,
            jointRanges=jr,
            restPoses=rs
        )
        if DEBUG:
            print(joint_angles)
        # 设置机器人关节角度
        set_angle(model_id, joint_angles)
        
    return joint_angles

def move_to_point(model_id, target_position, target_orientation=None, num_steps=100):
    
    ll = [-3.141592653589793, -1.5, -1.5, -1.7, -0.01, -0.01] 
    ul = [3.141592653589793, 1.5, 1.4, 1.97, 0.019, 0.019]
    jr = [ul[i] - ll[i] for i in range(len(ll))]
    rs = [0, 0, 0, 0, 0, 0]
    
    threshold = 1e-5
    max_iterations = int(1e4) # cpp-api need to ensure the type is int
    end_effector_index = 8
    
    assert target_position is not None
    startJointPositions = get_angle(model_id)
    endJointPositions =  p.calculateInverseKinematics(
        model_id,
        end_effector_index,
        target_position,
        targetOrientation=target_orientation,
        residualThreshold=threshold,
        maxNumIterations=max_iterations,
        lowerLimits=ll,
        upperLimits=ul,
        # jointRanges=jr,
        # restPoses=rs
        )
    
    trajectory = np.linspace(startJointPositions, endJointPositions, num_steps)
    
    
    for joint_angle in trajectory:
        if DEBUG:
            print(joint_angle)
        # 设置机器人关节角度
        set_angle(model_id, joint_angle)
        
        p.stepSimulation()
        time.sleep(1./240.)
        
    return get_angle(model_id)


if __name__ == "__main__":
    # p.connect(p.DIRECT)
    p.connect(p.GUI)
    p.setGravity(0, 0, 0)
    p.resetDebugVisualizerCamera(
        cameraDistance=1,  
        cameraYaw=50,     
        cameraPitch=-35,   
        cameraTargetPosition=[0, 0, 0]
    )
    
    model_id = p.loadURDF("./model/openx.urdf")
    # start = (0.286, 6.88214269644119e-22, 0.2045)
    target_position = (-1,1,0)  # 末端目标位置
    #  Euler angles
    target_orientation = p.getQuaternionFromEuler([0, 1, 1])  
    
    # move_to_point_line(model_id, target_position)
    move_to_point(model_id, target_position)
  
    while True:
        p.stepSimulation()
                
        if ord('i') in p.getKeyboardEvents():
            model_info(model_id)  
        if ord('q') in p.getKeyboardEvents():
            break
        
    # 清除物理引擎
    p.disconnect()
