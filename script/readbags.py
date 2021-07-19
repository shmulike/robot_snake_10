#!/usr/bin/env python3

from bagpy import bagreader
import bagpy
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt

b = bagreader('/home/robot-snake/rs_ws/src/robot_snake_10/bags/_2021-06-03-07-52-18.bag')
LASER_MSG = b.message_by_topic('/robot_snake_10/joint_cmd')
LASER_MSG
df_laser = pd.read_csv(LASER_MSG)
df_laser