import tkinter as tk
class RobotControlFrame(tk.Frame):
    def __init__(self,root,robot):
        super().__init__(root)
        self.config(width = 200,height=200)
        self.robot = robot
        self._init_control_buttons()

    def _init_control_buttons(self):
        self.grid_propagate(False)
        tk.Button(self,text='↑',command=self.move_robot_forward).grid(row=1,column=2)
        tk.Button(self,text='↓',command=self.move_robot_backward).grid(row=3,column=2)
        tk.Button(self,text='←',command=self.turn_robot_left, repeatdelay=1, repeatinterval=1).grid(row=2,column=1)
        tk.Button(self,text='→',command=self.turn_robot_right, repeatdelay=1, repeatinterval=1).grid(row=2,column=3)

    def move_robot_forward(self):   self.robot.move_forward()
    def move_robot_backward(self):  self.robot.move_backward()
    def turn_robot_left(self):      self.robot.turn_left()
    def turn_robot_right(self):     self.robot.turn_right()
