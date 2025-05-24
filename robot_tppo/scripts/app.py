from gui.control_frame import RobotControlFrame
from gui.nested_table import NestedTable
from robot import Robot

import tkinter as tk

class AppComponentData(tk.Tk):
    def __init__(self):
        super().__init__()
        self._init_robot()
        self._init_widgets()
        self.geometry('500x500')
        self.protocol("WM_DELETE_WINDOW", self.close_app)
        self.update()
        self.mainloop()

    def _init_robot(self):
        self.robot = Robot()

    def _init_widgets(self):
        self.table = NestedTable(self)
        self.control_frame = RobotControlFrame(self,self.robot)

        data = self.get_robot_data()
        
        self.table.build_tree(data)
        self.table.pack(expand=True, fill='both')
        self.control_frame.pack()

    def get_robot_data(self):
        data_lidar,data_imu,data_joint_states = self.robot.get_sensors_data()        

        while data_lidar == {} or data_imu == {} or data_joint_states == {}:
            data_lidar,data_imu,data_joint_states = self.robot.get_sensors_data()        

        data = {
            'joint_states': data_joint_states,
            'imu': data_imu,
            'lidar':{
                'ranges': data_lidar,
            }
        }
        return data

    def close_app(self):
        self.robot.destroy()
        self.quit()
        self.destroy()
        exit()

    def update(self):
        data = self.get_robot_data()
        self.table.update_values(data)  
        self.after(1,self.update)        

AppComponentData()
