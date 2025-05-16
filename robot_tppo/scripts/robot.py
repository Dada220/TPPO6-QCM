"""
robot class that allows for robot movement in 3D space
"""
import rclpy
from rclpy.node import Node # for msg control

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

from sensor_msgs.msg import Image,LaserScan,Imu
from cv_bridge import CvBridge
import cv2
from rclpy.executors import MultiThreadedExecutor
import threading
import time

class MessageBroker(Node):
    def __init__(self):
        super().__init__('img_tester')
        self.init_sub_img()
        self.init_pub_vel()

    def init_pub_vel(self):
        self.pub_vel = self.create_publisher(
                TwistStamped,
                '/cmd_vel',
                10
        )

    def init_sub_img(self):
        self.sub_img = self.create_subscription(
            Image,
            '/camera/image',
            self.camera_image_callback,
            10
        )
        self.bridge = CvBridge()

    def close_publishers(self):
        self.destroy_publisher(self.pub_vel)

    def close_subscribers(self):
        self.destroy_subscription(self.sub_img)

    def close(self):
        self.close_publishers()
        self.close_subscribers()
        self.destroy_node()

    def create_move_msg(self,direction):
        msg = TwistStamped()
        msg.header = Header()
        msg.header.frame_id = "base_link" # Example frame ID
        msg.twist = Twist()

        directions = {
            'forward':  ((1.0,0.0,0.0),(0.0,0.0,0.0)),
            'backward': ((-1.0,0.0,0.0),(0.0,0.0,0.0)),
            'left':     ((0.0,0.0,0.0),(0.0,0.0,1.0)),
            'right':    ((0.0,0.0,0.0),(0.0,0.0,-1.0)),
        }

        try:
            vec_lin,vec_ang = directions[direction]
            msg.twist.linear.x = vec_lin[0]
            msg.twist.linear.y = vec_lin[1]
            msg.twist.linear.z = vec_lin[2]
            msg.twist.angular.x = vec_ang[0]
            msg.twist.angular.y = vec_ang[1]
            msg.twist.angular.z = vec_ang[2]
        except:
            print('Unknown direction')
        return msg

    def publish_move_msg(self,direction):
        msg = self.create_move_msg(direction)
        print(msg)
        self.pub_vel.publish(msg)
    # -------------------------------------------------------
    def camera_image_callback(self,msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        #cv2.imshow('Camera Feed', cv_image)
        #cv2.waitKey(1)  # Refresh the image

class Robot:
    def __init__(self):
        rclpy.init(args=None)
        self._init_msg_broker()

    def _init_msg_broker(self):
        self.msg_broker = MessageBroker()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(self.msg_broker)
    
        self.spin_thread = threading.Thread(target=executor.spin, daemon=True)
        self.spin_thread.start()
        while not rclpy.ok():
            time.sleep(0.5)

    def destroy(self):
        self.msg_broker.close()
        rclpy.shutdown()
        self.spin_thread.join() # proper thread cleanup before the program exits. 

    def move_forward(self):  self.msg_broker.publish_move_msg('forward')
    def move_backward(self): self.msg_broker.publish_move_msg('backward')
    def turn_right(self):    self.msg_broker.publish_move_msg('left')
    def turn_left(self):     self.msg_broker.publish_move_msg('right')
