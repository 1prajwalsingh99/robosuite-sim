import numpy as np
import robosuite as suite
from robosuite.controllers import load_controller_config
import pandas as pd



controller_config = load_controller_config(default_controller="OSC_POSE")
controller_config["control_delta"] = False # This will make action inputs relate to global frame 
# print(controller_config)
df = pd.read_csv("/home/prajwal/Documents/robosuite/sampleInputsRobo - Sheet1.csv")
df1 = df[df["Label"] == "LABEL1"]
df2 = df[df["Label"] == "LABEL2"]
dfLst = [df1, df2]
# create environment instance
env = suite.make(
    env_name="Lift", # try with other tasks like "Stack" and "Door"
    robots="Panda",
    controller_configs=controller_config,  # try with other robots like "Sawyer" and "Jaco"
    has_renderer=False,
    has_offscreen_renderer=True,
    use_camera_obs=False,
    control_freq=200,                        # 20 hz control for applied actions
    horizon=2000,
)


tempLst = []
hashMap = {'Label': [0]*len(dfLst)*5*2000, 'relativeCycleTime': [0]*len(dfLst)*5*2000, 'robo_joint0_pos': [0]*len(dfLst)*5*2000, 
'robo_joint1_pos': [0]*len(dfLst)*5*2000, 'robo_joint2_pos': [0]*len(dfLst)*5*2000, 'robo_joint3_pos': [0]*len(dfLst)*5*2000, 
'robo_joint4_pos': [0]*len(dfLst)*5*2000, 'robo_joint5_pos': [0]*len(dfLst)*5*2000, 'eef_x': [0]*len(dfLst)*5*2000, 'eef_y': [0]*len(dfLst)*5*2000, 'eef_z': [0]*len(dfLst)*5*2000}

count = 0
for df in dfLst:
    for i in range(len(df)):
        action = [df.iloc[i]["x_pos"], df.iloc[i]["y_pos"], df.iloc[i]["z_pos"], np.pi, 0, 0, -0.5]
        env.reset()
        obs= None
        for j in range(2000):
            obs, reward, done, info = env.step(action)
            # env.render()
            robo_j0, robo_j1, robo_j2, robo_j3, robo_j4, robo_j5, grip = obs['robot0_joint_pos_cos']
            eff_x, eff_y, eff_z = obs['robot0_eef_pos']
            hashMap['Label'][count] = df.iloc[0]["Label"]
            hashMap['relativeCycleTime'][count] = j/200
            hashMap['robo_joint0_pos'][count] = robo_j0
            hashMap['robo_joint1_pos'][count] = robo_j1
            hashMap['robo_joint2_pos'][count] = robo_j2
            hashMap['robo_joint3_pos'][count] = robo_j3
            hashMap['robo_joint4_pos'][count] = robo_j4
            hashMap['robo_joint5_pos'][count] = robo_j5
            hashMap['eef_x'][count] = eff_x
            hashMap['eef_y'][count] = eff_y
            hashMap['eef_z'][count] = eff_z
            count += 1


newDf = pd.DataFrame(hashMap)
newDf.to_csv('output_V1.csv')