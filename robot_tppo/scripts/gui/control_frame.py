import tkinter as tk

class RobotControlFrame(tk.Frame):
    def __init__(self,root,robot):
        super().__init__(root)
        self.config(width = 200,height=200)
        self.robot = robot
        self._init_control_buttons()

    def _init_control_buttons(self):
        """Инициализация кнопок управления роботом"""
        self.grid_propagate(False)
        frame = tk.Frame(self)
        tk.Button(frame,text='↑',command=self.move_robot_forward,  repeatdelay=1, repeatinterval=1).grid(row=1,column=2)
        tk.Button(frame,text='↓',command=self.move_robot_backward, repeatdelay=1, repeatinterval=1).grid(row=3,column=2)
        tk.Button(frame,text='←',command=self.turn_robot_left,     repeatdelay=1, repeatinterval=1).grid(row=2,column=1)
        tk.Button(frame,text='→',command=self.turn_robot_right,    repeatdelay=1, repeatinterval=1).grid(row=2,column=3)

        tk.Label(self,text='Панель управления').pack()
        frame.pack()

    def move_robot_forward(self):   
        """Двигает робота вперёд"""
        self.robot.move_forward()

    def move_robot_backward(self):  
        """Двигает робота назад"""
        self.robot.move_backward()

    def turn_robot_left(self):      
        """Поворачивает робота налево"""
        self.robot.turn_left()

    def turn_robot_right(self):     
        """Поворачивает робота направо"""
        self.robot.turn_right()
