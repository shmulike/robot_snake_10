#!/usr/bin/env python3

# # df = pd.read_csv('/home/robot-snake/rs_ws/src/robot_snake_10/bags/exp4.csv')
# # # fig= px.line(df, x1='%time', y1='field.data2', title='exp4')
# # df1 = pd.read_csv('/home/robot-snake/rs_ws/src/robot_snake_10/bags/exp4realval.csv')
# #
# #
# # df.plot(kind='scatter', x='%time', y='field.data2')
# # plt.plot(x,y)
# # plt.show()
# #
# # df1.plot(kind='scatter', x1='%time', y1='field.data2')
# # plt.show()
# #
# # # fig.show()
# # # fig1 = px.line(df1, x='%time', y='field.data2', title='exp4')
#

import matplotlib.pyplot as plt
import pandas as pd
import mpld3

df = pd.read_csv('/home/robot-snake/rs_ws/src/robot_snake_10/bags/all.csv')
df1 = pd.read_csv('/home/robot-snake/rs_ws/src/robot_snake_10/bags/allrealval.csv')

fig, ax = plt.subplots()
df.plot(kind='line', x='%time', y='field.data0', ax=ax)
df.plot(kind='line', x='%time', y='field.data1', ax=ax)
df.plot(kind='line', x='%time', y='field.data2', ax=ax)
df.plot(kind='line', x='%time', y='field.data3', ax=ax)
df.plot(kind='line', x='%time', y='field.data4', ax=ax)
df.plot(kind='line', x='%time', y='field.data5', ax=ax)
df1.plot(kind='line', x='%time', y='field.data0', ax=ax)
df1.plot(kind='line', x='%time', y='field.data1', ax=ax)
df1.plot(kind='line', x='%time', y='field.data2', ax=ax)
df1.plot(kind='line', x='%time', y='field.data3', ax=ax)
df1.plot(kind='line', x='%time', y='field.data4', ax=ax)
df1.plot(kind='line', x='%time', y='field.data5', ax=ax)
plt.title("exp22")
mpld3.show(fig)
