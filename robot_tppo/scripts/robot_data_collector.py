from rclpy.node import Node 
from sensor_msgs.msg import Image,LaserScan,Imu,JointState

class ComponentDataCollector(Node):
    def __init__(self):
        super().__init__('component_data_collector')
        self._init_subscribers()
        self.data_lidar = {
            #"ranges": ''
        }
        self.data_imu = {
            "orientation":          {"x":'',"y":'',"z":'',"w":'',},
            "angular_velocity":     {"x":'',"y":'',"z":'',},
            "linear_acceleration":  {"x":'',"y":'',"z":'',}
        }
        self.data_joint_states = {}

    def _init_subscribers(self):
        self.sub_joint_states = self.create_subscription(JointState,'/joint_states',self.joint_states_callback,10)
        self.sub_imu          = self.create_subscription(Imu,'/imu',self.imu_callback,10)
        self.sub_lidar        = self.create_subscription(LaserScan,'/scan',self.lidar_callback,10)        
        self.sub_camera       = None

    def close_subscribers(self):
        """Закрытие всех инициализированных подписчиков перед закрытием брокера сообщении"""
        self.destroy_subscription(self.sub_joint_states) 
        self.destroy_subscription(self.sub_imu)          
        self.destroy_subscription(self.sub_lidar)        
        #self.destroy_subscription(self.sub_camera)       

    def close(self):
        """Закрытие брокера сообщении"""
        self.close_subscribers()    
        self.destroy_node()         

    def get_data(self):
        return (self.data_lidar,self.data_imu,self.data_joint_states)

    def lidar_callback(self,msg):
        """Callback при получении сообщения с лидара"""
        for i,data in enumerate(msg.ranges):
            self.data_lidar[f"{i}"] = data
        #self.data_lidar = msg.ranges 
        #print(list(msg.ranges))

    def imu_callback(self,msg):
        """Callback при получении сообщения с инерциального датчика"""
        o = msg.orientation
        a = msg.angular_velocity
        l = msg.linear_acceleration

        self.data_imu["orientation"]["x"] = o.x
        self.data_imu["orientation"]["y"] = o.y
        self.data_imu["orientation"]["z"] = o.z
        self.data_imu["orientation"]["w"] = o.w

        self.data_imu["angular_velocity"]["x"] = a.x
        self.data_imu["angular_velocity"]["y"] = a.y
        self.data_imu["angular_velocity"]["z"] = a.z

        self.data_imu["linear_acceleration"]["x"] = a.x
        self.data_imu["linear_acceleration"]["y"] = a.y
        self.data_imu["linear_acceleration"]["z"] = a.z

    def joint_states_callback(self,msg):
        """Callback при получении сообщения с соединении с колёсами"""
        for i in range(len(msg.name)):
            self.data_joint_states[msg.name[i]] = {
                    "position": msg.position[i],
                    "velocity": msg.velocity[i],
                    "effort"  : msg.effort[i]
            }
