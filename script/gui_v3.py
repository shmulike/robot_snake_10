import tkinter as tk
import tkinter.ttk as ttk
# import rospy
# from std_msgs.msg import Float64, Float32MultiArray


_max_PWM = 256
_N_joints = 10
_N_string = _N_joints * 2

class YScrolledFrame(tk.Frame):
    def __init__(self, parent, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        self.canvas = canvas = tk.Canvas(self, bg='white', relief='raised')
        canvas.grid(row=0, column=0, sticky='nsew')

        scroll = tk.Scrollbar(self, command=canvas.yview, orient=tk.VERTICAL)
        canvas.config(yscrollcommand=scroll.set)
        scroll.grid(row=0, column=1, sticky='nsew')

        self.content = tk.Frame(canvas)
        self.canvas.create_window(0, 0, window=self.content, anchor="nw")

        self.bind('<Configure>', self.on_configure)

    def on_configure(self, event):
        bbox = self.content.bbox('ALL')
        self.canvas.config(scrollregion=bbox)

class Notebook(ttk.Notebook):
    def __init__(self, parent, tab_labels):
        super().__init__(parent)

        self._tab = {}
        for text in tab_labels:
            self._tab[text] = YScrolledFrame(self)
            # layout by .add defaults to fill=tk.BOTH, expand=True
            self.add(self._tab[text], text=text, compound=tk.TOP)

    def tab(self, key):
        return self._tab[key].content

class App(tk.Tk):
    def __init__(self):
        super().__init__()

        self.win = tk.Tk()
        self.win.geometry("1000x600")
        self.win.title("Robot Snake 10")
        self.win.configure(bg='#E3DCA8')

        notebook = Notebook(self.win, ['Page 1', 'Page 2', 'Page 3'])
        notebook.grid(row=0, column=0, sticky='nsew')

        # Fill content, to see scroll action
        tab = notebook.tab('Page 1')
        for n in range(20):
            label = tk.Label(tab, text='Page 1 - Label {}'.format(n))
            label.grid()

if __name__ == '__main__':
    App().mainloop()