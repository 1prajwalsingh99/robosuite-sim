import numpy as np
import robosuite as suite
from robosuite.controllers import load_controller_config
import pandas as pd



controller_config = load_controller_config(default_controller="OSC_POSE")
controller_config["control_delta"] = False # This will make action inputs relate to global frame 
# print(controller_config)
df = pd.read_csv("sampleInputsRobo - Sheet1.csv")
df1 = df[df["Label"] == "LABEL1"]
df2 = df[df["Label"] == "LABEL2"]
dfLst = [df1, df2]
# create environment instance
env = suite.make(
    env_name="Lift", # try with other tasks like "Stack" and "Door"
    robots="Panda",
    controller_configs=controller_config,  # try with other robots like "Sawyer" and "Jaco"
    has_renderer=True,
    has_offscreen_renderer=False,
    use_camera_obs=False,
    control_freq=200,                        # 20 hz control for applied actions
    horizon=2000,
)



# reset the environment

 # [x,y,z,roll,pitch,yaw,gripper] not super sure on orientation tho 
# print(tempArr)

tempLst = []
hashMap = {'Label': [0]*len(dfLst)*5*2000, 'relativeCycleTime': [0]*len(dfLst)*5*2000, 'robo_joint0_pos': [0]*len(dfLst)*5*2000, 
'robo_joint1_pos': [0]*len(dfLst)*5*2000, 'robo_joint2_pos': [0]*len(dfLst)*5*2000, 'robo_joint3_pos': [0]*len(dfLst)*5*2000, 
'robo_joint4_pos': [0]*len(dfLst)*5*2000, 'robo_joint5_pos': [0]*len(dfLst)*5*2000, 'eef_x': [0]*len(dfLst)*5*2000, 'eef_y': [0]*len(dfLst)*5*2000, 'eef_z': [0]*len(dfLst)*5*2000}

finalPos = [0.1, 0.0, 1.0, np.pi, 0.0, 0.0, 0.5]
cube_pos = []

count = 0
for df in dfLst:
    for i in range(len(df)):
        # hashMap[df.iloc[0]["Label"]][i] = []
        
        action = [0.1, 0.0, 1.0, np.pi, 0.0, 0.0, 0.5]
        env.reset()
        obs= None
        initOBS = []
        for j in range(2000):

            if j == 0:
                obs, reward, done, info = env.step(action)
                cube_pos = obs['cube_pos']
                initOBS = obs['robot0_eef_pos']
                action = [cube_pos[0], initOBS[1], initOBS[2], np.pi, 0.0, 0.0, 0]

            elif 0 < j < 200:
                obs, reward, done, info = env.step(action)
            
            elif 200 <= j < 700:
                action = [cube_pos[0], cube_pos[1], initOBS[2], np.pi, 0.0, 0.0, 0]
                obs, reward, done, info = env.step(action)
            
            elif 700 <= j < 1000:
                action = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.05, np.pi, 0.0, 0.0, -0.5]
                obs, reward, done, info = env.step(action)
            elif 1000 <= j < 1300:
                action = [cube_pos[0] + 0.07, cube_pos[1] , cube_pos[2], np.pi, 0.0, 0.0, -0.9]
                obs, reward, done, info = env.step(action)
            elif 1300 <= j < 1600:
                action = [cube_pos[0] + 0.07    , cube_pos[1] , cube_pos[2], np.pi, 0.0, 0.0, 0.5]
                obs, reward, done, info = env.step(action)

            else:
                obs, reward, done, info = env.step(finalPos)
            
        
            env.render()
            robo_j0, robo_j1, robo_j2, robo_j3, robo_j4, robo_j5, grip = obs['robot0_joint_pos_cos']
            eff_x, eff_y, eff_z = obs['robot0_eef_pos']

            # tempSeries = pd.Series([df.iloc[0]["Label"], j/200, robo_x, robo_y, robo_z, roll, pitch, yaw, eff_x, eff_y, eff_z], name='temp')
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


            # tempOBSVar = obs[list(obs.keys())[0]]
            # tempLst.append(obs[list(obs.keys())[0]])
            count += 1


newDf = pd.DataFrame(hashMap)
newDf.to_csv('output_V1.csv')