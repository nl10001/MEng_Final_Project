#!/usr/bin/env python

import rospy
import time
import numpy as np
import Tkinter as tk
#from Tkinter import ttk

class MyGui(tk.Tk):
	def __init__(self, *args, **kwargs):
		tk.Tk.__init__(self, *args, **kwargs)
		self.title("test")
		container = tk.Frame(self, height=800, width=800)
		container.pack(side="top", fill="both", expand=True)

		container.grid_rowconfigure(0, weight=1)
		container.grid_columnconfigure(0, weight=1)
		

		self.frames = {}
		# we'll create the frames themselves later but let's add the components to the dictionary.
		for F in (MainPage, SidePage, CompletionScreen):
			frame = F(container, self)

			# the windows class acts as the root window for the frames.
			self.frames[F] = frame
			frame.grid(row=0, column=0, sticky="nsew")

		# Using a method to switch frames
		self.show_frame(MainPage)
		#self.geometry('350x200')
		#btn = tk.Button(self, text = 'Click me !', bd = '5',command = self.destroy)
		#btn.pack(side = 'top')

	#def add_stuff(self, tk.Frame):
	#	btn = Button(root, text = 'Click me !', bd = '5',command = root.destroy)
	#	btn.pack(side = 'top')

	def show_frame(self, cont):
		frame = self.frames[cont]
		# raises the current frame to the top
		frame.tkraise()

class MainPage(tk.Frame):
	def __init__(self, parent, controller):
		tk.Frame.__init__(self, parent)
		label = tk.Label(self, text="Main Page")
		label.pack(padx=10, pady=10)

		# We use the switch_window_button in order to call the show_frame() method as a lambda function
		switch_window_button = tk.Button(
			self,
			text="Go to the Side Page",
			command=lambda: controller.show_frame(SidePage),
		)
		switch_window_button.pack(side="bottom", fill=tk.X)


class SidePage(tk.Frame):
	def __init__(self, parent, controller):
		tk.Frame.__init__(self, parent)
		label = tk.Label(self, text="This is the Side Page")
		label.pack(padx=10, pady=10)

		switch_window_button = tk.Button(
			self,
			text="Go to the Completion Screen",
			command=lambda: controller.show_frame(CompletionScreen),
		)
		switch_window_button.pack(side="bottom", fill=tk.X)


class CompletionScreen(tk.Frame):
	def __init__(self, parent, controller):
		tk.Frame.__init__(self, parent)
		label = tk.Label(self, text="Completion Screen, we did it!")
		label.pack(padx=10, pady=10)
		switch_window_button = tk.Button(
			self,
			text="Return to menu",
			command=lambda: controller.show_frame(MainPage)
		)
		switch_window_button.pack(side="bottom", fill=tk.X)

if __name__ == '__main__':
	#try:				
	view = MyGui()
		#view.MyGui(root)
	view.mainloop()
#	except rospy.ROSInterruptException:
#		pass
