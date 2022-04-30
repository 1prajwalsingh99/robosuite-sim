from robosuite.models import MujocoWorldBase
from robosuite.models.robots import UR5e
from robosuite.models.grippers import gripper_factory
from robosuite.models.arenas import TableArena
from robosuite.models.objects import BallObject
from robosuite.utils.mjcf_utils import new_joint
import numpy as np
from mujoco_py import MjSim, MjViewer

world = MujocoWorldBase()
mujoco_robot = UR5e()

gripper = gripper_factory("Robotiq85Gripper")
mujoco_robot.add_gripper(gripper)

mujoco_robot.set_base_xpos([0, 0, 1])
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
tempArr = [0.05522334, -0.67813834, -1.22322997, -0.5260257, 0.1169881, 0.39127047,-0.88721667, 0.13166819]
for i in range(10000):
  for idx, val in enumerate(tempArr):
    tempArr[idx] += 0.5
  action = np.array(tempArr).any()
  sim.data.ctrl[:] = 0
  sim.step(action)
  viewer.render()



