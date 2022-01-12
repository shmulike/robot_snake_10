#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd
import mpld3
from sympy import Symbol, sin, cos
# Import math Library
import math
import scipy as sc
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
from scipy.signal import savgol_filter
from scipy import interpolate
from scipy.interpolate import CubicSpline

link_L = 160
figure_pwm = 1


# Rotation matrix around axis z
def Rz(tehta):
    return np.array([[np.cos(tehta), -np.sin(tehta), 0, 0],
                     [np.sin(tehta), np.cos(tehta), 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


# Rotation matrix around axis y
def Ry(tehta):
    return np.array([[np.cos(tehta), 0, np.sin(tehta), 0],
                     [0, 1, 0, 0],
                     [-np.sin(tehta), 0, np.cos(tehta), 0],
                     [0, 0, 0, 1]])


def A(x):
    return np.array([[1, 0, 0, x],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


# Path for csv files
Realangle_file_name = '/home/robot-snake/rs_ws/src/robot_snake_10/bags/joint_val108.csv'
SIN_file_name = '/home/robot-snake/rs_ws/src/robot_snake_10/bags/joint_cmd108.csv'
motor_file = '/home/robot-snake/rs_ws/src/robot_snake_10/bags/motor_cmd108.csv'
tension_file = '/home/robot-snake/rs_ws/src/robot_snake_10/bags/tension_val108.csv'

# Variables that contain the information from the .csv file
cur_angle = pd.read_csv(Realangle_file_name)
des_angle = pd.read_csv(SIN_file_name)
motor_data = pd.read_csv(motor_file)
tension_data = pd.read_csv(tension_file)
print("Finish opening CSV files")

fig, ax = plt.subplots()



# An array containing all the current angles
Realangle = -np.deg2rad(np.array(
    [[cur_angle.get('field.data0')], [cur_angle.get('field.data1')], [cur_angle.get('field.data2')],
     [cur_angle.get('field.data3')], [cur_angle.get('field.data4')], [cur_angle.get('field.data5')],
     [cur_angle.get('field.data6')], [cur_angle.get('field.data7')], [cur_angle.get('field.data8')],
     [cur_angle.get('field.data9')]])[:, 0, :])
# An array containing all the desired angles
SINangle = -np.deg2rad(np.array(
    [[des_angle.get('field.data0')], [des_angle.get('field.data1')], [des_angle.get('field.data2')],
     [des_angle.get('field.data3')], [des_angle.get('field.data4')], [des_angle.get('field.data5')],
     [des_angle.get('field.data6')], [des_angle.get('field.data7')], [des_angle.get('field.data8')],
     [des_angle.get('field.data9')]])[:, 0, :])
# An array containing all the motors
motor = np.array([[motor_data.get('field.data0')], [motor_data.get('field.data1')], [motor_data.get('field.data2')],
                  [motor_data.get('field.data3')], [motor_data.get('field.data4')], [motor_data.get('field.data5')],
                  [motor_data.get('field.data6')], [motor_data.get('field.data7')], [motor_data.get('field.data8')],
                  [motor_data.get('field.data9')], [motor_data.get('field.data10')], [motor_data.get('field.data11')],
                  [motor_data.get('field.data12')], [motor_data.get('field.data13')], [motor_data.get('field.data14')],
                  [motor_data.get('field.data15')], [motor_data.get('field.data16')], [motor_data.get('field.data17')],
                  [motor_data.get('field.data18')], [motor_data.get('field.data19')]])[:, 0, :]
tension = np.array(
    [[tension_data.get('field.data0')], [tension_data.get('field.data1')], [tension_data.get('field.data2')],
     [tension_data.get('field.data3')], [tension_data.get('field.data4')], [tension_data.get('field.data5')],
     [tension_data.get('field.data6')], [tension_data.get('field.data7')], [tension_data.get('field.data8')],
     [tension_data.get('field.data9')], [tension_data.get('field.data10')], [tension_data.get('field.data11')],
     [tension_data.get('field.data12')], [tension_data.get('field.data13')], [tension_data.get('field.data14')],
     [tension_data.get('field.data15')], [tension_data.get('field.data16')], [tension_data.get('field.data17')],
     [tension_data.get('field.data18')], [tension_data.get('field.data19')]])[:, 0, :]

for i in range(0, len(tension)):
    for j in range(1, tension.shape[1] - 1):
        if np.abs(tension[i, j]) > 50:
            tension[i, j] = np.mean([tension[i, j - 1], tension[i, j + 1]])

# An array containing the times for the current angle

cur_time = np.array([cur_angle.get('%time')]).T
des_time = np.array([des_angle.get('%time')]).T
cur_time_motor = np.array(motor_data.get('%time')).T
cur_time_tension = np.array(tension_data.get('%time')).T
start_time = min([cur_time[0][0], des_time[0][0], cur_time_motor[0], cur_time_tension[0]])
print(start_time)

# start_time = cur_time[0][0]
cur_time -= start_time
cur_time = cur_time * 10 ** -9  # Transition from nanosecond to second
# An array that contains the times for the desired angle
des_time -= start_time
des_time = des_time * 10 ** -9  # Transition from nanosecond to second

# An array containing the times for the current motor command
cur_time_motor -= start_time
cur_time_motor = cur_time_motor * 10 ** -9  # Transition from nanosecond to second

# An array containing the times for the current tension
cur_time_tension -= start_time
cur_time_tension = cur_time_tension * 10 ** -9  # Transition from nanosecond to second

# Interpolate to find the current angle for the times of the desired angle
new_cur_angle = np.array([np.interp(des_time.T[0, :], cur_time.T[0, :], Realangle.T[:, 0]),
                          np.interp(des_time.T[0, :], cur_time.T[0, :], Realangle.T[:, 1]),
                          np.interp(des_time.T[0, :], cur_time.T[0, :], Realangle.T[:, 2]),
                          np.interp(des_time.T[0, :], cur_time.T[0, :], Realangle.T[:, 3]),
                          np.interp(des_time.T[0, :], cur_time.T[0, :], Realangle.T[:, 4]),
                          np.interp(des_time.T[0, :], cur_time.T[0, :], Realangle.T[:, 5]),
                          np.interp(des_time.T[0, :], cur_time.T[0, :], Realangle.T[:, 6]),
                          np.interp(des_time.T[0, :], cur_time.T[0, :], Realangle.T[:, 7]),
                          np.interp(des_time.T[0, :], cur_time.T[0, :], Realangle.T[:, 8]),
                          np.interp(des_time.T[0, :], cur_time.T[0, :], Realangle.T[:, 9])])

new_cur_angle=np.array([savgol_filter(new_cur_angle[0, :], 137, 1, mode='interp'),
                        savgol_filter(new_cur_angle[1, :], 137, 1, mode='interp'),
                        savgol_filter(new_cur_angle[2, :], 137, 1, mode='interp'),
                        savgol_filter(new_cur_angle[3, :], 137, 1, mode='interp'),
                        savgol_filter(new_cur_angle[4, :], 137, 1, mode='interp'),
                        savgol_filter(new_cur_angle[5, :], 137, 1, mode='interp'),
                        savgol_filter(new_cur_angle[6, :], 137, 1, mode='interp'),
                        savgol_filter(new_cur_angle[7, :], 137, 1, mode='interp'),
                        savgol_filter(new_cur_angle[8, :], 137, 1, mode='interp'),
                        savgol_filter(new_cur_angle[9, :], 137, 1, mode='interp')
                        ])

q = np.array([[0], [0], [0], [1]])

all_norm = np.zeros(new_cur_angle.shape[0] + 1)
circle_move = np.zeros(4)
circle_move_real=np.zeros(4)
print("Start calculating link positions")
for real_theta, theta in zip(new_cur_angle.T, SINangle.T):
    # Displacement direction x in 200 mm
    A01 = np.array([[1, 0, 0, 200],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    q01 = A01 @ q - A01 @ q  # Calculate the direction vector between the desired and existing state

    # Calculate the position of the first JOINT tip in space
    A02 = A01 @ Ry(theta[0]) @ A(link_L)
    real_A02 = A01 @ Ry(real_theta[0]) @ A(link_L)

    q02 = A02 @ q - real_A02 @ q  # Calculate the direction vector between the desired and existing state

    # Calculate the position of the second JOINT tip in space
    A03 = A02 @ Rz(theta[1]) @ A(link_L)
    real_A03 = real_A02 @ Rz(real_theta[1]) @ A(link_L)
    # print(np.rad2deg(theta[1]-real_theta[1]))

    q03 = A03 @ q - real_A03 @ q  # Calculate the direction vector between the desired and existing state

    # Calculate the position of the third JOINT tip in space
    A04 = A03 @ Ry(theta[2]) @ A(link_L)
    real_A04 = real_A03 @ Ry(real_theta[2]) @ A(link_L)

    q04 = A04 @ q - real_A04 @ q  # Calculate the direction vector between the desired and existing state

    #  Calculate the position of the fourth JOINT tip in space
    A05 = A04 @ Rz(theta[3]) @ A(link_L)
    real_A05 = real_A04 @ Rz(real_theta[3]) @ A(link_L)

    q05 = A05 @ q - real_A05 @ q  # Calculate the direction vector between the desired and existing state

    #  Calculate the position of the fifth JOINT tip in space
    A06 = A05 @ Ry(theta[4]) @ A(link_L)
    real_A06 = real_A05 @ Ry(real_theta[4]) @ A(link_L)

    q06 = A06 @ q - real_A06 @ q  # Calculate the direction vector between the desired and existing state

    #  Calculate the position of the Sixth JOINT tip in space
    A07 = A06 @ Rz(theta[5]) @ A(link_L)
    real_A07 = real_A06 @ Rz(real_theta[5]) @ A(link_L)

    q07 = A07 @ q - real_A07 @ q  # Calculate the direction vector between the desired and existing state

    #  Calculate the position of the seventh JOINT tip in space
    A08 = A07 @ Ry(theta[6]) @ A(link_L)
    real_A08 = real_A07 @ Ry(real_theta[6]) @ A(link_L)

    q08 = A08 @ q - real_A08 @ q  # Calculate the direction vector between the desired and existing state

    #  Calculate the position of the eighth JOINT tip in space
    A09 = A08 @ Rz(theta[7]) @ A(link_L)
    real_A09 = real_A08 @ Rz(real_theta[7]) @ A(link_L)

    q09 = A09 @ q - real_A09 @ q  # Calculate the direction vector between the desired and existing state

    #  Calculate the position of the nine JOINT tip in space
    A010 = A09 @ Ry(theta[8]) @ A(link_L)
    real_A010 = real_A09 @ Ry(real_theta[8]) @ A(link_L)

    q010 = A010 @ q - real_A010 @ q  # Calculate the direction vector between the desired and existing state

    #  Calculate the position of the tenth JOINT tip in space
    A011 = A010 @ Rz(theta[9]) @ A(link_L)
    real_A011 = real_A010 @ Rz(real_theta[9]) @ A(link_L)

    q011 = A011 @ q - real_A011 @ q  # Calculate the direction vector between the desired and existing state

    circle_move = np.vstack((circle_move, np.array(real_A011  @ q).T))
    circle_move_real = np.vstack((circle_move_real, np.array(A011 @ q).T))

    # Calculate the norm of each distance vector
    all_norm = np.vstack((all_norm, np.linalg.norm(np.array(
        [q01[:-1], q02[:-1], q03[:-1], q04[:-1], q05[:-1], q06[:-1], q07[:-1], q08[:-1], q09[:-1], q010[:-1],
         q011[:-1]])[:, :, 0], axis=1)))


#  Plot
print("end-effector: x:avg:{:.2f}\t std:{:.2f}".format(np.mean(circle_move[:, 0]), np.std(circle_move[:, 0])))
print("start plotting")

M = len(des_time[:])

plt.figure(1)

plt.plot(des_time[0:M, 0], all_norm[0:M, 1], 'b', label='Joint1')
plt.plot(des_time[0:M, 0], all_norm[0:M, 2], 'g', label='Joint2')
plt.plot(des_time[0:M, 0], all_norm[0:M, 3], 'r', label='Joint3')
plt.plot(des_time[0:M, 0], all_norm[0:M, 4], 'k', label='Joint4')
plt.plot(des_time[0:M, 0], all_norm[0:M, 5], 'c', label='Joint5')
plt.plot(des_time[0:M, 0], all_norm[0:M, 6], 'y', label='Joint6')
plt.plot(des_time[0:M, 0], all_norm[0:M, 7], 'crimson', label='Joint7')
plt.plot(des_time[0:M, 0], all_norm[0:M, 8], 'steelblue', label='Joint8')
plt.plot(des_time[0:M, 0], all_norm[0:M, 9], 'gold', label='Joint9')
plt.plot(des_time[0:M, 0], all_norm[0:M, 10], 'mediumspringgreen', label='Joint10')
plt.xlabel("Time [sec]")
plt.ylabel("Position error [mm]")
plt.legend(['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint7', 'joint8', 'joint9', 'joint10'])

# # Convert all angles rad -> deg
new_cur_angle = new_cur_angle * 180 / np.pi
SINangle = SINangle * 180 / np.pi

plt.figure(2)
plt.subplot(3, 2, 1)
plt.plot(des_time[0:M, 0], new_cur_angle[0, :], '-b', des_time[0:M, 0], SINangle[0, :], '--b')
plt.ylabel("${\Theta_1}$ deg", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-7, 7])
plt.subplot(3, 2, 3)
plt.plot(des_time[0:M, 0], new_cur_angle[1, :], '-g', des_time[0:M, 0], SINangle[1, :], '--g')
plt.ylabel("${\Theta_2}$ deg", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-7, 7])
plt.subplot(3, 2, 5)
plt.plot(des_time[0:M, 0], new_cur_angle[2, :], 'firebrick')
plt.plot(des_time[0:M, 0], SINangle[2, :], 'firebrick', linestyle='--')
plt.ylabel("${\Theta_3}$ deg", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-7, 7])
plt.xlabel("Time sec", fontsize=14, fontweight='bold', labelpad=20)
plt.subplot(3, 2, 2)
plt.plot(des_time[0:M, 0], new_cur_angle[3, :], '-k', des_time[0:M, 0], SINangle[3, :], '--k')
plt.ylabel("${\Theta_4}$ deg ", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-7, 7])
plt.subplot(3, 2, 4)
plt.plot(des_time[0:M, 0], new_cur_angle[4, :], 'purple')
plt.plot(des_time[0:M, 0], SINangle[4, :], 'purple', linestyle='--')
plt.ylabel("${\Theta_5}$ deg ", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-7, 7])
plt.subplot(3, 2, 6)
plt.plot(des_time[0:M, 0], new_cur_angle[5, :], 'saddlebrown')
plt.plot(des_time[0:M, 0], SINangle[5, :], 'saddlebrown', linestyle='--')
plt.ylabel("${\Theta_6}$ deg ", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.yticks(fontsize=14)
plt.xticks(fontsize=14)
plt.grid()
plt.ylim([-7, 7])
plt.xlabel("Time sec", fontsize=14, fontweight='bold', labelpad=20)

plt.figure(3)
plt.subplot(2, 2, 1)
plt.plot(des_time[0:M, 0], new_cur_angle[6, :], 'purple')
plt.plot(des_time[0:M, 0], SINangle[6, :], 'purple', linestyle='--')
plt.ylabel("${\Theta_7}$ deg ", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-7, 7])
plt.subplot(2, 2, 3)
plt.plot(des_time[0:M, 0], new_cur_angle[7, :], 'steelblue')
plt.plot(des_time[0:M, 0], SINangle[7, :], 'steelblue', linestyle='--')
plt.ylabel("${\Theta_8}$ deg ", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.yticks(fontsize=14)
plt.xticks(fontsize=14)
plt.grid()
plt.ylim([-7, 7])
plt.xlabel("Time sec", fontsize=14, fontweight='bold', labelpad=20)
plt.subplot(2, 2, 2)
plt.plot(des_time[0:M, 0], new_cur_angle[8, :], 'gold')
plt.plot(des_time[0:M, 0], SINangle[8, :], 'gold', linestyle='--')
plt.ylabel("${\Theta_9}$ deg ", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.yticks(fontsize=14)
plt.xticks(fontsize=14)
plt.grid()
plt.ylim([-7, 7])
plt.subplot(2, 2, 4)
plt.plot(des_time[0:M, 0], new_cur_angle[9, :], 'mediumspringgreen')
plt.plot(des_time[0:M, 0], SINangle[9, :], 'mediumspringgreen', linestyle='--')
plt.ylabel("${\Theta_10}$ deg ", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.yticks(fontsize=14)
plt.xticks(fontsize=14)
plt.grid()
plt.ylim([-7, 7])
plt.xlabel("Time sec", fontsize=14, fontweight='bold', labelpad=20)
plt.tight_layout()
# plt.savefig('Joints1.eps', format='eps')
# plt.savefig('Joints1.svg', format='svg')
plt.savefig('Joints_circle.png', format='png')
#
#
y_max = 2
plt.figure(4)
plt.subplot(5, 2, 1)
plt.plot(des_time[0:M, 0], new_cur_angle[0, :] - SINangle[0, :], '-b')
plt.ylabel("${\Theta_1}$ deg", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-y_max, y_max])
plt.subplot(5, 2, 3)
plt.plot(des_time[0:M, 0], new_cur_angle[1, :] - SINangle[1, :], '-g')
plt.ylabel("${\Theta_2}$ deg", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-y_max, y_max])
plt.subplot(5, 2, 5)
plt.plot(des_time[0:M, 0], new_cur_angle[2, :] - SINangle[2, :], 'firebrick')
plt.ylabel("${\Theta_3}$ deg", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-y_max, y_max])
plt.xlabel("Time sec", fontsize=14, fontweight='bold', labelpad=20)
plt.subplot(5, 2, 7)
plt.plot(des_time[0:M, 0], new_cur_angle[3, :] - SINangle[3, :], '-k')
plt.ylabel("${\Theta_4}$ deg ", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-y_max, y_max])
plt.subplot(5, 2, 9)
plt.plot(des_time[0:M, 0], new_cur_angle[4, :] - SINangle[4, :], 'purple')
plt.ylabel("${\Theta_5}$ deg ", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-y_max, y_max])
plt.subplot(5, 2, 2)
plt.plot(des_time[0:M, 0], new_cur_angle[5, :] - SINangle[5, :], 'saddlebrown')
plt.ylabel("${\Theta_6}$ deg ", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.yticks(fontsize=14)
plt.xticks(fontsize=14)
plt.grid()
plt.ylim([-y_max, y_max])
plt.subplot(5, 2, 4)
plt.plot(des_time[0:M, 0], new_cur_angle[6, :] - SINangle[6, :], 'crimson')
plt.ylabel("${\Theta_7}$ deg ", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.yticks(fontsize=14)
plt.xticks(fontsize=14)
plt.grid()
plt.ylim([-y_max, y_max])
plt.subplot(5, 2, 6)
plt.plot(des_time[0:M, 0], new_cur_angle[7, :] - SINangle[7, :], 'steelblue')
plt.ylabel("${\Theta_8}$ deg ", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.yticks(fontsize=14)
plt.xticks(fontsize=14)
plt.grid()
plt.ylim([-y_max, y_max])
plt.subplot(5, 2, 8)
plt.plot(des_time[0:M, 0], new_cur_angle[8, :] - SINangle[8, :], 'gold')
plt.ylabel("${\Theta_9}$ deg ", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.yticks(fontsize=14)
plt.xticks(fontsize=14)
plt.grid()
plt.ylim([-y_max, y_max])
plt.subplot(5, 2, 10)
plt.plot(des_time[0:M, 0], new_cur_angle[9, :] - SINangle[9, :], 'mediumspringgreen')
plt.ylabel("${\Theta_10}$ deg ", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.yticks(fontsize=14)
plt.xticks(fontsize=14)
plt.grid()
plt.ylim([-y_max, y_max])
plt.xlabel("Time sec", fontsize=14, fontweight='bold', labelpad=20)
#
# # plt.legend(['Joint1', 'Joint2', 'Joint3','Joint4', 'Joint5','Joint6'])
# # plt.show()

plt.figure(5)
plt.subplot(5, 2, 1)
plt.plot(cur_time_motor, motor[0, :], 'blue', cur_time_motor, motor[1, :], 'skyblue')
plt.ylabel("JOINT_1  PWM", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['motor1 +', 'motor1  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-220, 220])
plt.subplot(5, 2, 3)
plt.plot(cur_time_motor, motor[2, :], 'g', cur_time_motor, motor[3, :], 'lightgreen')
plt.ylabel("JOINT_2 PWM", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['motor2 +', 'motor2  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-220, 220])
plt.subplot(5, 2, 5)
plt.plot(cur_time_motor, motor[4, :], 'firebrick', cur_time_motor, motor[5, :], 'salmon')
plt.ylabel("JOINT_3 PWM", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['motor3 +', 'motor3  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-220, 220])
plt.subplot(5, 2, 7)
plt.plot(cur_time_motor, motor[6, :], 'k', cur_time_motor, motor[7, :], 'dimgray')
plt.ylabel("JOINT_4 PWM", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['motor4 +', 'motor4  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-220, 220])
plt.subplot(5, 2, 9)
plt.plot(cur_time_motor, motor[8, :], 'purple', cur_time_motor, motor[9, :], 'mediumorchid')
plt.ylabel("JOINT_5 PWM", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['motor5 +', 'motor5  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-220, 220])
plt.xlabel("Time sec", fontsize=14, fontweight='bold', labelpad=20)
plt.subplot(5, 2, 2)
plt.plot(cur_time_motor, motor[10, :], 'saddlebrown', cur_time_motor, motor[11, :], 'sandybrown')
plt.ylabel("JOINT_6 PWM", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['motor6 +', 'motor6  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-220, 220])
plt.subplot(5, 2, 4)
plt.plot(cur_time_motor, motor[12, :], 'crimson', cur_time_motor, motor[13, :], 'palevioletred')
plt.ylabel("JOINT_7 PWM", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['motor7 +', 'motor7  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-220, 220])
plt.subplot(5, 2, 6)
plt.plot(cur_time_motor, motor[14, :], 'steelblue', cur_time_motor, motor[15, :], 'deepskyblue')
plt.ylabel("JOINT_8 PWM", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['motor8 +', 'motor8  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-220, 220])
plt.subplot(5, 2, 8)
plt.plot(cur_time_motor, motor[16, :], 'gold', cur_time_motor, motor[17, :], 'olive')
plt.ylabel("JOINT_9 PWM", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['motor9 +', 'motor9  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-220, 220])
plt.subplot(5, 2, 10)
plt.plot(cur_time_motor, motor[18, :], 'mediumspringgreen', cur_time_motor, motor[19, :], 'darkolivegreen')
plt.ylabel("JOINT_10 PWM", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['motor10 +', 'motor10  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-220, 220])
plt.xlabel("Time sec", fontsize=14, fontweight='bold', labelpad=20)
plt.tight_layout()
# plt.savefig('Joints2.eps', format='eps')
# plt.savefig('Joints2.svg', format='svg')
plt.savefig('circle_motor.png', format='png')

tension_max = 40
tension_min = -40
plt.figure(6)
plt.subplot(3, 2, 1)
plt.plot(cur_time_tension, tension[0, :], 'blue', cur_time_tension, tension[1, :], 'skyblue')
plt.axhline(y=1.5, color='skyblue', linestyle='-')
plt.ylabel("Joint_1 N", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['tension1 +', 'tension1  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([tension_min, tension_max])
plt.subplot(3, 2, 3)
plt.plot(cur_time_tension, tension[2, :], 'g', cur_time_tension, tension[3, :], 'lightgreen')
plt.axhline(y=1.5, color='lightgreen', linestyle='-')
plt.ylabel("Joint_2 N", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['tension2 +', 'tension2  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([tension_min, tension_max])
plt.subplot(3, 2, 5)
plt.plot(cur_time_tension, tension[4, :], 'firebrick', cur_time_tension, tension[5, :], 'salmon')
plt.axhline(y=1.5, color='salmon', linestyle='-')
plt.ylabel("Joint_3 N", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['tension3 +', 'tension3  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([tension_min, tension_max])
plt.xlabel("Time [sec]", fontsize=14, fontweight='bold', labelpad=20)
plt.subplot(3, 2, 2)
plt.plot(cur_time_tension, tension[6, :], 'k', cur_time_tension, tension[7, :], 'dimgray')
plt.axhline(y=1.5, color='dimgray', linestyle='-')
plt.ylabel("Joint_4 N", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['tension4 +', 'tension4  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([tension_min, tension_max])
plt.subplot(3, 2, 4)
plt.plot(cur_time_tension, tension[8, :], 'purple', cur_time_tension, tension[9, :], 'mediumorchid')
plt.axhline(y=1.5, color='mediumorchid', linestyle='-')
plt.ylabel("Joint_5 N", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['tension5 +', 'tension5  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([tension_min, tension_max])
plt.subplot(3, 2, 6)
plt.plot(cur_time_tension, tension[10, :], 'saddlebrown', cur_time_tension, tension[11, :], 'sandybrown')
plt.axhline(y=1.5, color='sandybrown', linestyle='-')
plt.ylabel("Joint_6 N", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['tension6 +', 'tension6  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([tension_min, tension_max])
plt.xlabel("Time [sec]", fontsize=14, fontweight='bold', labelpad=20)
plt.figure(7)
plt.subplot(3, 2, 1)
plt.plot(cur_time_tension, tension[12, :], 'crimson', cur_time_tension, tension[13, :], 'palevioletred')
plt.axhline(y=1.5, color='palevioletred', linestyle='-')
plt.ylabel("Joint_7 N", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['tension7 +', 'tension7  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([tension_min, tension_max])
plt.subplot(3, 2, 3)
plt.plot(cur_time_tension, tension[14, :], 'steelblue', cur_time_tension, tension[15, :], 'deepskyblue')
plt.axhline(y=1.5, color='deepskyblue', linestyle='-')
plt.ylabel("Joint_8 N", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['tension8 +', 'tension8  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([tension_min, tension_max])
plt.xlabel("Time [sec]", fontsize=14, fontweight='bold', labelpad=20)
plt.subplot(3, 2, 2)
plt.plot(cur_time_tension, tension[16, :], 'gold', cur_time_tension, tension[17, :], 'olive')
plt.axhline(y=1.5, color='olive', linestyle='-')
plt.ylabel("Joint_9 N", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['tension9 +', 'tension9  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([tension_min, tension_max])
plt.subplot(3, 2, 4)
plt.plot(cur_time_tension, tension[18, :], 'mediumspringgreen', cur_time_tension, tension[19, :], 'darkolivegreen')
plt.axhline(y=1.5, color='darkolivegreen', linestyle='-')
plt.ylabel("Joint_10 N", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['tension10 +', 'tension10  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([tension_min, tension_max])
plt.xlabel("Time [sec]", fontsize=14, fontweight='bold', labelpad=20)
plt.tight_layout()
# plt.savefig('Joints3.eps', format='eps')
# plt.savefig('Joints3.svg', format='svg')
plt.savefig('circle_tension.png', format='png')

plt.figure(9)
plt.subplot(4,1,1)
plt.plot(des_time[0:M, 0], all_norm[0:M, 5], 'purple', label='Joint5')
plt.grid()
plt.subplot(4,1,2)
plt.plot(des_time[0:M, 0], new_cur_angle[4, :], 'purple')
plt.plot(des_time[0:M, 0], SINangle[4, :], 'purple', linestyle='--')
plt.ylabel("${\Theta_5}$ deg ", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-7, 7])
plt.subplot(4,1,3)
plt.plot(des_time[0:M, 0], new_cur_angle[4, :] - SINangle[4, :], 'purple')
plt.ylabel("${\Theta_5}$ deg ", fontweight='bold', labelpad=30, fontsize=14)
plt.legend(['current', 'desirable'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
plt.grid()
plt.ylim([-y_max, y_max])
plt.subplot(4,1,4)
plt.plot(cur_time_tension, tension[8, :], 'purple', cur_time_tension, tension[9, :], 'mediumorchid')
plt.axhline(y=0.5, color='mediumorchid', linestyle='--')
plt.ylabel("Joint_5 N", fontweight='bold', labelpad=15, fontsize=14)
plt.legend(['tension5 +', 'tension5  -'], loc='upper right', fontsize='large')
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)

fig3 = plt.figure(10)
ax = fig3.add_subplot(111, projection='3d')
x_circle= savgol_filter(circle_move[:, 0], 51, 3)
y_circle= savgol_filter(circle_move[:, 1], 51, 3)
z_circle= savgol_filter(circle_move[:, 2], 51, 3)
ax.scatter(x_circle, y_circle, z_circle, linewidths=0.1)
ax.scatter(circle_move_real[:,0],circle_move_real[:,1],circle_move_real[:,2], linewidths=0.1)
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()

# plt.figure(2)
# plt.plot(des_time[0:M, 0], new_cur_angle[0, :]-SINangle[0, :], '-b')
# plt.plot(des_time[0:M, 0], new_cur_angle[1, :]-SINangle[1, :], '-g')
# plt.plot(des_time[0:M, 0], new_cur_angle[2, :]-SINangle[2, :], '-r')
# plt.plot(des_time[0:M, 0], new_cur_angle[3, :]-SINangle[3, :], '-k')
# plt.plot(des_time[0:M, 0], new_cur_angle[4, :]-SINangle[4, :], '-c')
# plt.plot(des_time[0:M, 0], new_cur_angle[5, :]-SINangle[5, :], '-y')
# plt.legend(['Joint1', 'Joint2', 'Joint3','Joint4', 'Joint5','Joint6'])
plt.show()
