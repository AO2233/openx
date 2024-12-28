from tools import OpenX, PortManager, config
from simulation import OpenX_sim
import pybullet as p
import time

if __name__ == "__main__":
    port_manager = PortManager(config.DEVICENAME)
    arm = OpenX(port_manager)
    arm.set_all_toqure(False)