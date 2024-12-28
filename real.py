from tools import OpenX, PortManager, config
from simulation import OpenX_sim
import pybullet as p
import time

SIM=False

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
        
    if SIM:
        op = arm_sim.move_to_point_line((0.2, 0.2, 0.2))
        for joint_angle in op:
            arm_sim.set_joint_sim(joint_angle)
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
            print(arm_sim.get_joint_sim())
        # while True:
        #     p.stepSimulation()
    
    if not SIM:
        port_manager=PortManager(config.DEVICENAME)
        arm_real = OpenX(port_manager)   
        
        arm_real.set_all_toqure(True) 
        init_joint = arm_real.get_joint_real()
        init_joint += [0]
        arm_sim.set_joint_sim(init_joint)
        #start = (0.286, 6.88214269644119e-22, 0.21)

        op = arm_sim.move_to_point_line((0.28, 0, 0.21))
        for joint_angle in op:
            arm_real.set_joint_real(joint_angle)
            time.sleep(0.3)
        
        op = arm_sim.move_to_point_line((0.15, -0.2, 0.2))
        for joint_angle in op:
            arm_real.set_joint_real(joint_angle)
            time.sleep(0.3)
        
        op = arm_sim.move_to_point_line((0.3, -0.2, 0.3))
        for joint_angle in op:
            arm_real.set_joint_real(joint_angle)
            time.sleep(0.3)
        
        op = arm_sim.move_to_point_line(((0.28, 0, 0.21)))
        for joint_angle in op:
            arm_real.set_joint_real(joint_angle)
            time.sleep(0.3)
            
     

        print(arm_real.get_joint_real())
               
    p.disconnect()

