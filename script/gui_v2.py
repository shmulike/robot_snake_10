#!/usr/bin/env python3
# license removed for brevity

import tkinter as tk
from tkinter import ttk
import rospy
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import numpy as np

_max_PWM = 256
_N_joints = 4
_N_string = _N_joints * 2


class APP:

    def __init__(self):
        # ROS
        self.motor_pwm_pub = rospy.Publisher("/robot_snake_4/motor_cmd", Int32MultiArray, queue_size=10)
        rospy.Subscriber("/robot_snake_1/tension_val", Float32MultiArray, self.tension_val_update)
        rospy.Subscriber("/robot_snake_1/joint_val", Float32MultiArray, self.joint_val_update)

        self.win = tk.Tk()
        self.win.geometry("800x600")
        self.win.title("Robot Snake 10")
        self.win.configure(bg='#E3DCA8')

        # Create main window
        self.tabControl = ttk.Notebook(self.win)

        # Create tabs for the windows
        self.tab1 = ttk.Frame(self.tabControl)
        self.tab2 = ttk.Frame(self.tabControl)
        self.tab3 = ttk.Frame(self.tabControl)
        # Set tabs names
        self.tabControl.add(self.tab1, text='Motor Control')
        self.tabControl.add(self.tab2, text='Joint Control')
        self.tabControl.add(self.tab3, text='About')
        self.tabControl.pack(expand=1, fill="both")


        # Create vector of sliders, button, lables, ...
        # self.slider_pwm = [0 for x in range(_N_strins)]
        self.slider_pwm = [[0 for i in range(2)] for j in range(_N_string)]
        self.joint_val_text = [0 for x in range(_N_joints)]
        self.joint_val = [0 for x in range(_N_joints)]
        # self.tension_label = [0 for x in range(_N_strins)]
        self.tension_label = [[0 for i in range(2)] for j in range(_N_string)]
        self.tension_text = [[0 for i in range(2)] for j in range(_N_string)]

        # Create tab for about
        ttk.Label(self.tab3, text="Welcome to Robot Snake 10 Control\nShmulik Edelman\nshmulike@post.bgu.ac.il").grid(
            column=0, row=0, padx=30, pady=50)

        # Create tab view for about
        for i in range(_N_joints):
            # joints angle value
            self.joint_val_text[i] = tk.StringVar()
            self.joint_val[i] = ttk.Label(self.tab1, textvariable=self.joint_val_text[i])
            self.joint_val[i].grid(column=0, row=i * 2, padx=30)
            self.joint_val_text[i].set("Joint {}: {} deg".format(i + 1, 0.123))

            # motor command pwn
            for j in range(2):
                ii = i * 2 + j
                self.slider_pwm[i][j] = tk.Scale(self.tab1, length=300, from_=-_max_PWM, to=_max_PWM,
                                                 orient=tk.HORIZONTAL,
                                                 tickinterval=_max_PWM / 2)
                self.slider_pwm[i][j].grid(column=1, row=ii)
                # self.slider_pwm[i].bind("<ButtonRelease-1>", self.slider_pwm_val)
                self.slider_pwm[i][j].bind("<ButtonRelease-1>",
                                           lambda event, arg1=i, arg2=j: self.slider_pwm_reset(event, arg1, arg2))
                self.slider_pwm[i][j].bind("<B1-Motion>",
                                           lambda event, arg1=i, arg2=j: self.slider_pwm_change(event, arg1, arg2))
                # self.slider_pwm[i].pack()

                self.tension_text[i][j] = tk.StringVar()
                self.tension_label[i][j] = ttk.Label(self.tab1, textvariable=self.tension_text[i][j])
                self.tension_label[i][j].grid(column=2, row=ii, padx=30)
                # self.tension_label[i].pack()
                self.tension_text[i][j].set("String #{}: Waite for connection".format(j + 1))

        self.win.mainloop()

        while not rospy.is_shutdown():
            self.rate.sleep()

    def slider_pwm_change(self, event, i, j):
        # print(self.slider_pwm[i][j].get())
        arr = np.zeros(_N_string)
        arr[i*2+j] = self.slider_pwm[i][j].get()
        self.send_motor_cmd(arr)

    def slider_pwm_reset(self, event, i, j):
        self.slider_pwm[i][j].set(0)
        arr = np.zeros(_N_string)
        self.send_motor_cmd(arr)
        return

    def joint_val_update(self, msg):
        for i in range(_N_joints):
            self.joint_val_text[i].set("Joint {}: {:.3f} deg".format(i + 1, msg.data[i]))

    def tension_val_update(self, msg):
        for i in range(_N_string):
            row = int(np.floor(i/2))
            col = int(i % 2)
            if (msg.data[i] > 0):
                text = "String #{}: {:.2f} Kg".format(i + 1, msg.data[i])
            else:
                text = "String #{}: {:.2f} Kg -- Error".format(i + 1, msg.data[i])
            self.tension_text[row][col].set(text)

    def send_motor_cmd(self, arr):
        msg = Int32MultiArray(data=arr.astype(int))
        self.motor_pwm_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node("gui", anonymous=True)
    rate = rospy.Rate(100)
    APP()
    rospy.spin()

