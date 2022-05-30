import imp
import mujoco_py
import numpy as np
from robosuite.models import MujocoWorldBase
from robosuite.models.robots import Panda
from robosuite.robots.single_arm import SingleArm
from robosuite.models.grippers import gripper_factory
from robosuite.models.arenas import TableArena
from robosuite.models.objects import BallObject
from robosuite.utils.mjcf_utils import new_joint
from robosuite.controllers import load_controller_config
from mujoco_py import MjSim, MjViewer



controller_config = load_controller_config(default_controller="OSC_POSE")
controller_config["control_delta"] = False 
world = MujocoWorldBase()
# qpos = UR5e().init_qpos()

mujoco_robot  = Panda()

# mujoco_robot = UR5e()

gripper = gripper_factory('PandaGripper')
mujoco_robot.add_gripper(gripper)

mujoco_robot.set_base_xpos([0.8, 0, 0.7])
world.merge(mujoco_robot)


mujoco_arena = TableArena()
mujoco_arena.set_origin([0.8, 0, 0])
world.merge(mujoco_arena)

sphere = BallObject(
    name="sphere",
    size=[0.04],
    rgba=[0, 0.5, 0.5, 1]).get_obj()
sphere.set('pos', '1.0 0 1.0')
world.worldbody.append(sphere)


model = world.get_model(mode="mujoco_py")

sim = MjSim(model)
viewer = MjViewer(sim)
viewer.vopt.geomgroup[0] = 0 # disable visualization of collision mesh
action = [0.1, 0.0, 1.0, np.pi, 0.0, 0.0, 0.5]
for i in range(10000):
  sim.data.ctrl[:] = 0
  sim.step(action)
  viewer.render()