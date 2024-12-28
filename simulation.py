import time
import numpy as np
import pybullet as p

# DEBUG = True
DEBUG = False
STEP=10

class OpenX_sim:
    def __init__(self, model_id):
        self.model_id = model_id
        self.ll = [-3.141592653589793, -1.5, -1.5, -1.7, -0.01, -0.01]
        self.ul = [3.141592653589793, 1.5, 1.4, 1.97, 0.019, 0.019]
        self.jr = [self.ul[i] - self.ll[i] for i in range(len(self.ll))]
        self.rs = [0, 0, 0, 0, 0, 0]
        self.threshold = 1e-5
        self.max_iterations = int(1e4)  # cpp-api need to ensure the type is int
        self.end_effector_index = 8
    
    def model_info(self):
        num_joints = p.getNumJoints(self.model_id)
        for joint_index in range(num_joints):
            joint_info = p.getJointInfo(self.model_id, joint_index)
            link_state = p.getLinkState(self.model_id, joint_index)

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

    def set_joint_sim(self, angle_list) -> list:
        # index [1,2,3,4,5,6]
        for i in range(1, 7):
            p.resetJointState(self.model_id, i, angle_list[i - 1])
        return angle_list

    def get_joint_sim(self) -> list:
        angle_list = []
        for i in range(1, 7):
            angle_list.append(p.getJointState(self.model_id, i)[0])
        return angle_list
    
    def move_to_point_line(self, target_position, target_orientation=None, num_steps=STEP):
        if target_orientation is not None:
            print("target_orientation is not None, it's dengerous to use move_to_point_line.")

        assert target_position is not None
        now_position = p.getLinkState(self.model_id, self.end_effector_index)[0]
        trajectory = np.linspace(now_position, target_position, num_steps)

        if DEBUG:
            for i in range(len(trajectory) - 1):
                p.addUserDebugLine(trajectory[i], trajectory[i + 1], [1, 0, 0], lineWidth=2)

        operation_list=[]
        for target_point in trajectory:
            joint_angles = p.calculateInverseKinematics(
                self.model_id,
                self.end_effector_index,
                target_point,
                targetOrientation=target_orientation,
                residualThreshold=self.threshold,
                maxNumIterations=self.max_iterations,
                lowerLimits=self.ll,
                upperLimits=self.ul,
                jointRanges=self.jr,
                restPoses=self.rs,
            )
            if DEBUG:
                print(joint_angles)
            operation_list.append(joint_angles)
        return operation_list

    def move_to_point(self, target_position, target_orientation=None, num_steps=STEP):
        assert target_position is not None
        startJointPositions = self.get_joint_sim()
        endJointPositions = p.calculateInverseKinematics(
            self.model_id,
            self.end_effector_index,
            target_position,
            targetOrientation=target_orientation,
            residualThreshold=self.threshold,
            maxNumIterations=self.max_iterations,
            lowerLimits=self.ll,
            upperLimits=self.ul,
            # jointRanges=jr,
            # restPoses=rs
        )
        trajectory = np.linspace(startJointPositions, endJointPositions, num_steps)
        if DEBUG:
            for joint_angle in trajectory:
                print(joint_angle)    
        return trajectory

if __name__ == "__main__":
    p.connect(p.DIRECT)
    # p.connect(p.GUI)
    p.setGravity(0, 0, 0)
    p.resetDebugVisualizerCamera(
        cameraDistance=1, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0, 0, 0]
    )

    model_id = p.loadURDF("./model/openx.urdf")
    arm_sim = OpenX_sim(model_id)
    
    # start = (0.286, 6.88214269644119e-22, 0.2045)
    target_position = (0.2, 0.2, 0.21)  # end effector position
    #  Euler angles
    # target_orientation = p.getQuaternionFromEuler([0, 1, 1])

    op = arm_sim.move_to_point_line(target_position)
    # arm_sim.move_to_point_line(target_position)

    for joint_angle in op:
        arm_sim.set_joint_sim(joint_angle)
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

    while True:
        p.stepSimulation()
        if ord("i") in p.getKeyboardEvents():
            arm_sim.model_info(model_id)
        if ord("q") in p.getKeyboardEvents():
            break

    p.disconnect()
