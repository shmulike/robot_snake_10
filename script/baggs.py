#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd
import mpld3
from sympy import Symbol, sin, cos
# Import math Library
import math
from sympy.physics.vector import init_vprinting
init_vprinting(use_latex='mathjax', pretty_print=False)
import numpy as np
from sympy.matrices import Matrix
import sympy as sp
from sympy import *
from sympy.physics.mechanics import dynamicsymbols
from sympy.tensor.array import Array
from numpy import array
from numpy.linalg import norm
from matplotlib import pyplot as plt
from scipy import interpolate
from scipy.interpolate import CubicSpline

link_L = 160
# Rotation matrix around axis z
def Rz(tehta):
    return np.array([[np.cos(tehta),-np.sin(tehta),0,0],
            [np.sin(tehta),np.cos(tehta),0,0],
            [0,0,1,0],
            [0,0,0,1]])

# Rotation matrix around axis y
def Ry(tehta):
    return np.array([[np.cos(tehta),0,np.sin(tehta),0],
            [0,1,0,0],
            [-np.sin(tehta), 0, np.cos(tehta), 0],
            [0,0,0,1]])

def A(x):
    return np.array([[1,0,0,x],
            [0,1,0,0],
            [0, 0, 1, 0],
            [0,0,0,1]])

# Path for csv files
Realangle_file_name = '/home/robot-snake/rs_ws/src/robot_snake_10/bags/all.csv'
SIN_file_name = '/home/robot-snake/rs_ws/src/robot_snake_10/bags/allrealval.csv'

# Variables that contain the information from the .csv file
cur_angle = pd.read_csv(Realangle_file_name)
des_angle = pd.read_csv(SIN_file_name)
print("Finish opening CSV files")

fig, ax = plt.subplots()

# An array containing all the current angles
Realangle=-np.deg2rad(np.array([[cur_angle.get('field.data0')] ,[cur_angle.get('field.data1')], [cur_angle.get('field.data2')] ,[cur_angle.get('field.data3')], [cur_angle.get('field.data4')] ,[cur_angle.get('field.data5')]])[:,0,:])
#An array containing all the desired angles
SINangle=-np.deg2rad(np.array([[des_angle.get('field.data0')], [des_angle.get('field.data1')],[des_angle.get('field.data2')],[des_angle.get('field.data3')],[des_angle.get('field.data4')],[des_angle.get('field.data5')]])[:,0,:])

# An array containing the times for the current angle
cur_time = np.array([cur_angle.get('%time')]).T
start_time = cur_time[0][0]
cur_time -= start_time
cur_time= cur_time*10**-9 # Transition from nanosecond to second
# An array that contains the times for the desired angle
des_time = np.array([des_angle.get('%time')]).T
des_time -= start_time
des_time=des_time*10**-9 # Transition from nanosecond to second




# Interpolate to find the current angle for the times of the desired angle
new_cur_angle = np.array([np.interp( des_time.T[0,:],cur_time.T[0,:], Realangle.T[:,0]), np.interp( des_time.T[0,:],cur_time.T[0,:], Realangle.T[:,1]), np.interp( des_time.T[0,:],cur_time.T[0,:], Realangle.T[:,2]),np.interp( des_time.T[0,:],cur_time.T[0,:], Realangle.T[:,3]),np.interp( des_time.T[0,:],cur_time.T[0,:], Realangle.T[:,4]),np.interp( des_time.T[0,:],cur_time.T[0,:], Realangle.T[:,5])])

data_cur = np.hstack([cur_time, Realangle.T])
data_des = np.hstack([des_time, SINangle.T])

q=np.array([[0],[0],[0],[1]])

all_norm=np.zeros(new_cur_angle.shape[0]+1)
print("Start calculating link positions")
for theta , real_theta in zip(new_cur_angle.T, SINangle.T):

    # Displacement direction x in 200 mm
    A01= np.array([[1,0,0,200],
            [0,1,0,0],
            [0, 0, 1, 0],
            [0,0,0,1]])

    q01=A01@q-A01@q # Calculate the direction vector between the desired and existing state

# Calculate the position of the first JOINT tip in space
    A02=A01@Ry(theta[0])@A(link_L)
    real_A02=A01@Ry(real_theta[0])@A(link_L)

    q02=A02@q-real_A02@q # Calculate the direction vector between the desired and existing state

# Calculate the position of the second JOINT tip in space
    A03=A02@Rz(theta[1])@A(link_L)
    real_A03=real_A02@Rz(real_theta[1])@A(link_L)
    # print(np.rad2deg(theta[1]-real_theta[1]))

    q03=A03@q-real_A03@q # Calculate the direction vector between the desired and existing state

# Calculate the position of the third JOINT tip in space
    A04=A03@Ry(theta[2])@A(link_L)
    real_A04=real_A03@Ry(real_theta[2])@A(link_L)

    q04=A04@q-real_A04@q # Calculate the direction vector between the desired and existing state

#  Calculate the position of the fourth JOINT tip in space
    A05=A04@Rz(theta[3])@A(link_L)
    real_A05=real_A04@Rz(real_theta[3])@A(link_L)

    q05=A05@q-real_A05@q # Calculate the direction vector between the desired and existing state

#  Calculate the position of the fifth JOINT tip in space
    A06=A05@Ry(theta[4])@A(link_L)
    real_A06=real_A05@Ry(real_theta[4])@A(link_L)

    q06=A06@q-real_A06@q # Calculate the direction vector between the desired and existing state

#  Calculate the position of the Sixth JOINT tip in space
    A07=A06@Rz(theta[5])@A(link_L)
    real_A07=real_A06@Rz(real_theta[5])@A(link_L)

    q07=A07@q-real_A07@q # Calculate the direction vector between the desired and existing state

# Calculate the norm of each distance vector
    all_norm=np.vstack((all_norm,np.linalg.norm(np.array([q01[:-1],q02[:-1],q03[:-1],q04[:-1],q05[:-1],q06[:-1], q07[:-1]])[:,:,0],axis=1)))

#  Plot
print("start plotting")
M=35816
plt.figure(1)
plt.plot(des_time[0:M, 0], all_norm[0:M,1],'b', label='Joint1')
plt.plot(des_time[0:M, 0], all_norm[0:M,2],'g',label='Joint2')
plt.plot(des_time[0:M, 0], all_norm[0:M,3],'r',label='Joint3')
plt.plot(des_time[0:M, 0], all_norm[0:M,4],'k',label='Joint4')
plt.plot(des_time[0:M, 0], all_norm[0:M,5],'c',label='Joint5')
plt.plot(des_time[0:M, 0], all_norm[0:M,6],'y' ,label='Joint6')
plt.xlabel("Time [sec]")
plt.ylabel("Position error [mm]")
plt.legend(['Joint1', 'Joint2', 'Joint3','Joint4', 'Joint5','Joint6'])

# Convert all angles rad -> deg
new_cur_angle = new_cur_angle*180/np.pi
SINangle = SINangle*180/np.pi

plt.figure(2)
plt.plot(des_time[0:M, 0], new_cur_angle[0,:],'-b', des_time[0:M, 0], SINangle[0,:],'--b')
plt.plot(des_time[0:M, 0], new_cur_angle[1,:],'-g', des_time[0:M, 0], SINangle[1,:],'--g')
plt.plot(des_time[0:M, 0], new_cur_angle[2,:],'-r', des_time[0:M, 0], SINangle[2,:],'--r')
plt.plot(des_time[0:M, 0], new_cur_angle[3,:],'-k', des_time[0:M, 0], SINangle[3,:],'--k')
plt.plot(des_time[0:M, 0], new_cur_angle[4,:],'-c', des_time[0:M, 0], SINangle[4,:],'--c')
plt.plot(des_time[0:M, 0], new_cur_angle[5,:],'-y', des_time[0:M, 0], SINangle[5,:],'--y')
# plt.legend(['Joint1', 'Joint2', 'Joint3','Joint4', 'Joint5','Joint6'])
# plt.show()

plt.figure(3)
plt.plot(des_time[0:M, 0], new_cur_angle[0, :]-SINangle[0, :], '-b')
plt.plot(des_time[0:M, 0], new_cur_angle[1, :]-SINangle[1, :], '-g')
plt.plot(des_time[0:M, 0], new_cur_angle[2, :]-SINangle[2, :], '-r')
plt.plot(des_time[0:M, 0], new_cur_angle[3, :]-SINangle[3, :], '-k')
plt.plot(des_time[0:M, 0], new_cur_angle[4, :]-SINangle[4, :], '-c')
plt.plot(des_time[0:M, 0], new_cur_angle[5, :]-SINangle[5, :], '-y')
# plt.legend(['Joint1', 'Joint2', 'Joint3','Joint4', 'Joint5','Joint6'])
plt.show()