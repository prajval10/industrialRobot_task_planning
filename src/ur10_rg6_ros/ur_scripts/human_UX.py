#!/usr/bin/env python
import time
import threading
import human_interface as HI

try: import tkinter
except ImportError:
    import Tkinter as tkinter
    import ttk
    import Queue as queue
else:
    from tkinter import ttk
    import queue

class GUI_Core(object):

    def __init__(self):
        self.root = tkinter.Tk()

        self.int_var = tkinter.IntVar()
        progbar = ttk.Progressbar(self.root, maximum=4)
        # associate self.int_var with the progress value
        progbar['variable'] = self.int_var
        progbar.pack()

        self.label = ttk.Label(self.root, text='0/4')
        self.label.pack()

        self.b_start = ttk.Button(self.root, text='Start')
        self.b_start['command'] = self.start_thread
        self.b_start.pack()

    def start_thread(self):
        self.b_start['state'] = 'disable'
        self.int_var.set(0) # empty the Progressbar
        self.label['text'] = '0/4'
        # create then start a secondary thread to run arbitrary()
        self.secondary_thread = threading.Thread(target=arbitrary)
        self.secondary_thread.start()
        # check the Queue in 50ms
        self.root.after(50, self.check_que)

    def check_que(self):
        while True:
            try: x = que.get_nowait()
            except queue.Empty:
                self.root.after(25, self.check_que)
                break
            else: # continue from the try suite
                self.label['text'] = '{}/4'.format(x)
                self.int_var.set(x)
                if x == 4:
                    self.b_start['state'] = 'normal'
                    break



def arbitrary():
    

    func_a()
    que.put(1)
    func_b()
    que.put(2)
    func_c()
    que.put(3)
    func_d()
    que.put(4)

que = queue.Queue()
gui = GUI_Core() # see GUI_Core's __init__ method
gui.root.mainloop()