from tools import OpenX
from simulation import OpenX_sim
import pybullet as p
import time

SIM=True

if __name__ == "__main__":

    if SIM:
        p.connect(p.GUI)
        p.setGravity(0, 0, 0)
        p.setRealTimeSimulation(1)
        p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0, 0, 0])
        
    else:    
        p.connect(p.DIRECT)
        p.setGravity(0, 0, 0)

    model_id = p.loadURDF("./model/openx.urdf")
    arm_sim = OpenX_sim(model_id)
    
    
    op = arm_sim.move_to_point_line((0.2, 0.2, 0.2))
    # op = arm_sim.move_to_point((0.2, 0.2, 0.2))
    
    if SIM:
        for joint_angle in op:
            arm_sim.set_joint_sim(joint_angle)
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
            print(arm_sim.get_joint_sim())
        # while True:
        #     p.stepSimulation()
    
    if not SIM:
        arm_real = OpenX()
        
        init_joint = arm_real.get_joint_real()
        init_joint += [0]
        arm_sim.set_joint_sim(init_joint)
        
        op = arm_sim.move_to_point_line((0, 0, 0))
        for joint_angle in op:
            arm_real.set_joint_real(joint_angle)
        # arm_real.set_joint_real([0, 0, 0, 0, 0])
        for joint_angle in op:
            arm_real.set_joint_real(joint_angle)
        print(arm_real.get_joint_real())
               
    p.disconnect()

