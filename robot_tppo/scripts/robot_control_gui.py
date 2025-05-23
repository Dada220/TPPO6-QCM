from robot import Robot
import tkinter as tk

class AppRobotControl(tk.Tk):
    def __init__(self):
        super().__init__()
        self.robot = Robot()
        self.title('Robot control GUI')
        self.geometry('500x500')
        self.protocol("WM_DELETE_WINDOW", self.close_app)
        self._init_control_buttons()
        self.mainloop()

    def _init_control_buttons(self):
        frame = tk.Frame(self,width=200,height=200)
        frame.grid_propagate(False)
        tk.Button(frame,text='↑',command=self.move_robot_forward).grid(row=1,column=2)
        tk.Button(frame,text='↓',command=self.move_robot_backward).grid(row=3,column=2)
        tk.Button(frame,text='←',command=self.turn_robot_left, repeatdelay=1, repeatinterval=1).grid(row=2,column=1)
        tk.Button(frame,text='→',command=self.turn_robot_right, repeatdelay=1, repeatinterval=1).grid(row=2,column=3)
        tk.Button(frame,text=' ',command=self.plug,).grid(row=2,column=2)
        tk.Label(self,text='Клавиши управления').pack(anchor=tk.NW,fill=tk.X)
        frame.pack()

    def plug(self):                 print('Middle button: not implemented')
    def move_robot_forward(self):   self.robot.move_forward()
    def move_robot_backward(self):  self.robot.move_backward()
    def turn_robot_left(self):      self.robot.turn_left()
    def turn_robot_right(self):     self.robot.turn_right()

    def close_app(self):
        self.quit()
        self.destroy()

AppRobotControl()
