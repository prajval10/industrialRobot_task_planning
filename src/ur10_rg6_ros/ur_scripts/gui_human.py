#!/usr/bin/env pythons
import Tkinter as tk
import sys

class Handover:
    def __init__(self, master):
        self.master = master
        self.frame = tk.Frame(self.master, height = 700, width = 850)
        self.frame.pack()
        
        self.label1 = tk.Label(self.frame, 
                text="""Please Place the part in robot grasping area:""",
                justify = tk.CENTER,
                padx = 20, font=("Helvetica", 14))
        self.label1.pack()
        self.label1.place(x=190, y= 80)
        
    def close_windows(self):
        self.master.destroy()
    

class PickPlace:
    def __init__(self, master):
        self.master = master
        self.frame = tk.Frame(self.master)
        self.quitButton = tk.Button(self.frame, text = 'Quit', width = 25, command = self.close_windows)
        self.quitButton.pack()
        self.frame.pack()

    def new_window(self):
        self.newWindow = tk.Toplevel(self.master)
        # self.app = Demo2(self.newWindow)

    # def close_windows(self):
    #     self.master.destroy()

def main_gui(): 
    root = tk.Tk()
    app = Handover(root)
    root.mainloop()