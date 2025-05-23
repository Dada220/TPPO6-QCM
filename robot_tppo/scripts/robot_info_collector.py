import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image,LaserScan,Imu
from sensor_msgs.msg import JointState
import math

class ComponentDataCollector(Node):
    def __init__(self):
        super().__init__('component_data_collector')
        self.sub_joint_states = self.create_subscription(JointState,'/joint_states',self.joint_states_callback,10)
        self.sub_imu          = self.create_subscription(Imu,'/imu',self.imu_callback,10)
        self.sub_lidar        = self.create_subscription(LaserScan,'/scan',self.lidar_callback,10)        
        self.sub_camera       = None

    def lidar_callback(self,msg):
        """Callback при получении сообщения с лидара"""
        print('='*40)
        print(msg.ranges)

    def imu_callback(self,msg):
        """Callback при получении сообщения с инерциального датчика"""
        o = msg.orientation
        a = msg.angular_velocity
        l = msg.linear_acceleration
        print('='*40)
        print(f"orienation:\nx:{o.x}\ny:{o.y}\nz:{o.z}\nw:{o.w}")
        print('-'*40)
        print(f"angular_velocity:\nx:{a.x}\ny:{a.y}\nz:{a.z}")
        print('-'*40)
        print(f"linear_acceleration:\nx:{l.x}\ny:{l.y}\nz:{l.z}")

    def joint_states_callback(self,msg):
        """Callback при получении сообщения с соединении с колёсами"""
        print('='*40)
        for i in range(len(msg.name)):
            name = msg.name[i]
            position = msg.position[i] # position is expressed in radians in case it needs to be converted into degrees
            velocity = msg.velocity[i]
            effort   = msg.effort[i]
            print(f"{name}:\npos: {position} \nvel: {velocity}\n eff: {effort}")

def main(args=None):
    rclpy.init(args=args)
    info_collector = ComponentDataCollector()
    rclpy.spin(info_collector)
    info_collector.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
