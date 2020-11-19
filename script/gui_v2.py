import tkinter as tk
from tkinter import ttk

_max_PWM = 256
_N_joints = 4
_N_strins = _N_joints*2
class APP:
    def __init__(self):
        self.win = tk.Tk()
        self.win.geometry("1000x600")
        self.win.title("Robot Snake 10")

        # Create vector of sliders, button, lables, ...
        self.slider_pwm = [0 for x in range(_N_strins)]
        self.tension_label = [0 for x in range(_N_strins)]
        self.var = [0 for x in range(_N_strins)]

        # # Create tab for about
        # ttk.Label(self.win, text="Welcome to Robot Snake 10 Control\nShmulik Edelman\nshmulike@post.bgu.ac.il").grid(column=0, row=0, padx=30, pady=50)

        # Create tab view for about
        for i in range(_N_strins):
            self.slider_pwm[i] = ttk.Scale(self.win, length=300, from_=-_max_PWM, to=_max_PWM, orient=tk.HORIZONTAL, tickinterval=_max_PWM/2)
            # self.slider_pwm[i].grid(column=0, row=0,  sticky=tk.NSEW)
            # self.slider_pwm[i]

            # self.slider_pwm[i].bind("<ButtonRelease-1>", self.slider_pwm_val)
            self.slider_pwm[i].bind("<ButtonRelease-1>", lambda event, arg1=i: self.slider_pwm_reset(event, arg1))
            self.slider_pwm[i].bind("<B1-Motion>", lambda event, arg1=i: self.slider_pwm_change(event, arg1))
            self.slider_pwm[i].pack()

            self.var[i] = tk.StringVar()
            self.tension_label[i] = ttk.Label(self.win, textvariable=self.var[i])
            self.tension_label[i].pack()
            self.var[i].set("String tension: {} Kg".format(1.23))


        self.win.mainloop()


    def slider_pwm_change(self, event, i):
        return
        #print(self.slider_pwm[i].get())
        # self.slider_pwm[i].set(0)

    def slider_pwm_reset(self, event, i):
        print(self.slider_pwm[i].get())
        self.slider_pwm[i].set(0)



app = APP()

