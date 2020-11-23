import tkinter as tk
from tkinter import ttk
import rospy
from std_msgs.msg import Float64, Float32MultiArray
import numpy as np

_max_PWM = 256
_N_joints = 4
_N_strins = _N_joints * 2


class APP:
    def __init__(self):
        self.win = tk.Tk()
        self.win.geometry("1000x600")
        self.win.title("Robot Snake 10")

        # Create main window
        tabControl = ttk.Notebook(self.win)

        # Create tabs for the windows
        self.tab1 = ttk.Frame(tabControl)
        self.tab2 = ttk.Frame(tabControl)
        self.tab3 = ttk.Frame(tabControl)
        # Set tabs names
        tabControl.add(self.tab1, text='Motor Control')
        tabControl.add(self.tab2, text='Joint Control')
        tabControl.add(self.tab3, text='About')
        tabControl.pack(expand=1, fill="both")

        # Create vector of sliders, button, lables, ...
        # self.slider_pwm = [0 for x in range(_N_strins)]
        self.slider_pwm = [[0 for i in range(2)] for j in range(_N_strins)]
        self.joint_val_text = [0 for x in range(_N_joints)]
        self.joint_val = [0 for x in range(_N_joints)]
        # self.tension_label = [0 for x in range(_N_strins)]
        self.tension_label = [[0 for i in range(2)] for j in range(_N_strins)]
        self.tension_text = [[0 for i in range(2)] for j in range(_N_strins)]

        # Create tab for about
        ttk.Label(self.tab3, text="Welcome to Robot Snake 10 Control\nShmulik Edelman\nshmulike@post.bgu.ac.il").grid(
            column=0, row=0, padx=30, pady=50)

        # Create tab view for about
        for i in range(_N_joints):
            # joints angle value
            self.joint_val_text[i] = tk.StringVar()
            self.joint_val[i] = ttk.Label(self.tab1, textvariable=self.joint_val_text[i])
            self.joint_val[i].grid(column=0, row=i*2, padx=30)
            self.joint_val_text[i].set("Joint {}: {} deg".format(i+1, 0.123))

            # motor command pwn
            for j in range(2):
                ii = i*2+j
                self.slider_pwm[i][j] = tk.Scale(self.tab1, length=300, from_=-_max_PWM, to=_max_PWM, orient=tk.HORIZONTAL,
                                            tickinterval=_max_PWM / 2)
                self.slider_pwm[i][j].grid(column=1, row=ii)
                # self.slider_pwm[i].bind("<ButtonRelease-1>", self.slider_pwm_val)
                self.slider_pwm[i][j].bind("<ButtonRelease-1>", lambda event, arg1=i, arg2=j: self.slider_pwm_reset(event, arg1, arg2))
                self.slider_pwm[i][j].bind("<B1-Motion>", lambda event, arg1=i, arg2=j: self.slider_pwm_change(event, arg1, arg2))
                # self.slider_pwm[i].pack()

                self.tension_text[i][j] = tk.StringVar()
                self.tension_label[i][j] = ttk.Label(self.tab1, textvariable=self.tension_text[i][j])
                self.tension_label[i][j].grid(column=2, row=ii, padx=30)
                # self.tension_label[i].pack()
                self.tension_text[i][j].set("String #{}: {} Kg".format(j+1, 1.23))

        self.win.mainloop()

        # ROS
        self.motor_pub = rospy.Publisher("/motor_pub", Float64, queue_size=10)
        self.joint_sub = rospy.Subscriber("/joint_sub", Float32MultiArray, self.joint_val_update)

    def slider_pwm_change(self, event, i, j):
        return
        # print(self.slider_pwm[i].get())
        # self.slider_pwm[i].set(0)

    def slider_pwm_reset(self, event, i, j):
        print(self.slider_pwm[i][j].get())
        self.slider_pwm[i][j].set(0)

    def joint_val_update(self, data):
        for i in data:
            print(i)
        # self.joint_val_text[i].set("Joint {}: {} deg".format(i + 1, 0.123))



if __name__ == '__main__':
    rospy.init_node('main_app')
    app = APP()
    rospy.spin()
