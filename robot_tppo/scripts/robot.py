import rclpy
import threading
import time
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node # for msg control
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped,Twist
from sensor_msgs.msg import Image,LaserScan,Imu

class MessageBroker(Node):
    def __init__(self):
        super().__init__('img_tester')
        self._init_sub_img()
        self._init_pub_vel()

    def _init_pub_vel(self):
        """Initialization of movement publisher"""
        self.pub_vel = self.create_publisher(
                TwistStamped,
                '/cmd_vel',
                10
        )
    def _init_sub_img(self):
        """Initialization of camera subscriber"""
        self.sub_img = self.create_subscription(
            Image,
            '/camera/image',
            self.camera_image_callback,
            10
        )
        self.bridge = CvBridge()

    def close_publishers(self):
        """Closing all initialized publishers before closing MessageBroker"""
        self.destroy_publisher(self.pub_vel)

    def close_subscribers(self):
        """Closing all initialized subscribers before closing MessageBroker"""
        self.destroy_subscription(self.sub_img)

    def close(self):
        """Properly closing MessageBroker"""
        self.close_publishers()     
        self.close_subscribers()    
        self.destroy_node()         

    def create_move_msg(self,direction):
        """
        Creates move TwistStamped message with set direction
        Args:
            direction(str): direction in which robot is moving
        Returns:
            msg(TwistStamped): movement message
        """
        msg = TwistStamped()
        # Because message is TwistStamped it consists of Header and Twist message
        msg.header = Header()
        msg.header.frame_id = "base_link" # Example frame ID

        msg.twist = Twist()
        # Fixed robot directions
        directions = {
            'forward':  ((1.0,0.0,0.0),(0.0,0.0,0.0)),
            'backward': ((-1.0,0.0,0.0),(0.0,0.0,0.0)),
            'left':     ((0.0,0.0,0.0),(0.0,0.0,1.0)),
            'right':    ((0.0,0.0,0.0),(0.0,0.0,-1.0)),
        }
        # Creating Twist part of the TwistStamped message
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
            msg.twist.linear.x = 0
            msg.twist.linear.y = 0
            msg.twist.linear.z = 0
            msg.twist.angular.x = 0
            msg.twist.angular.y = 0
        return msg
    
    def publish_move_msg(self,direction):
        """
        Publishes move message with set direction
        Args:
            direction(str): direction in which robot is moving
        """
        msg = self.create_move_msg(direction)
        self.move_msg = msg
        time.sleep(1)   # Adding delay for move command to finish its execution
        self.pub_vel.publish(msg)

    def camera_image_callback(self,msg):
        """
        Detects images in camera
        Args:
            msg: message from /camera/image topic 
        """
        # Converting image to black and white
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # WARNING: In Ubuntu cv2 version is 4.6.0 
        # Same code but in 4.11.0 version:
        #   aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        #   parameters = cv2.aruco.DetectorParameters()
        #   detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        #   corners, ids, rejected = detector.detectMarkers(image)

        # Marker detection on image
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(image, aruco_dict, parameters=parameters)
        print("Detected markers:", ids)

class Robot:
    def __init__(self):
        rclpy.init(args=None)
        self._init_msg_broker()

    def _init_msg_broker(self):
        """Initialization of message broker"""
        self.msg_broker = MessageBroker()
        # Launching broker in separate thread to be able to use it 
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(self.msg_broker)
        self.spin_thread = threading.Thread(target=executor.spin, daemon=True)
        self.spin_thread.start()

        while not rclpy.ok():
            time.sleep(0.5)

    def destroy(self):
        """Robot desctruction"""
        self.msg_broker.close() 
        rclpy.shutdown()
        self.spin_thread.join() # proper thread cleanup before the program exits. 

    def move_forward(self): 
        """Publishes a 'forward' movement command to the message broker."""
        self.msg_broker.publish_move_msg('forward')
    def move_backward(self): 
        """Publishes a 'backward' movement command to the message broker."""
        self.msg_broker.publish_move_msg('backward')
    def turn_right(self):    
        """Publishes a 'right' movement command to the message broker."""
        self.msg_broker.publish_move_msg('right')
    def turn_left(self):     
        """Publishes a 'left' movement command to the message broker."""
        self.msg_broker.publish_move_msg('left')
