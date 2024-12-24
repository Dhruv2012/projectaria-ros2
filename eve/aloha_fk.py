import os
import torch
import pytorch_kinematics as pk
import numpy as np

class AlohaFK:
    def __init__(self):
        # urdf_path = os.path.join(
        #     os.path.dirname(egomimic.__file__), "resources/model.urdf"
        # )
        # Get the directory of the current script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        # urdf_path = os.path.join(script_dir, "model.urdf")
        urdf_path = "/home/rl2-bonjour/interbotix_ws/src/aloha-ros2/eve/model.urdf"
        self.chain = pk.build_serial_chain_from_urdf(
            open(urdf_path).read(), "vx300s/ee_gripper_link"
        )

    def fk(self, qpos):
        # if isinstance(qpos, np.ndarray):
        #     print("npa rray")
        #     qpos = torch.from_numpy(qpos).type(torch.DoubleTensor)
        
        # return self.chain.forward_kinematics(qpos, end_only=True).get_matrix()[:, :3, 3]
        return self.chain.forward_kinematics(qpos, end_only=True).get_matrix()[0]