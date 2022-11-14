#!/usr/bin/env python

import rospy
import time
import numpy as np
import Tkinter as tk

class ArucoGui(tk.Tk):
	def __init__(self, *args, **kwargs):
		tk.Tk.__init__(self, *args, **kwargs)
		self.title("Action Cards")
		self.geometry('200x200')
		terminate_btn = tk.Button(self, text = 'Terminated!', bd = '10',command = self.shutdownhook)
		pause_btn = tk.Button(self, text = 'Pause!', bd = '10',command = self.pause_tb)
		resume_btn = tk.Button(self, text = 'Resumed!', bd = '10',command = self.)
		start_btn.grid(row=0,column=0)
		stop_btn.grid(row=1,column=0)
		pause_btn.grid(row=2,column=0)
		
		#MainPage.tkraise()
		

if __name__ == '__main__':
	#try:				
	view = ArucoGui()
		#view.MyGui(root)
	view.mainloop()
