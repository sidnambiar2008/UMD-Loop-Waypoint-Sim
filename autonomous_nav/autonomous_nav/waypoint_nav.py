import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # Create a publisher that publishes Twist messages for cmd_vel
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        # Initialize starting odometry position
        self.x = 0.0
        self.y = 0.0
        self.target_x = 4.0
        self.target_y = 3.0
        self.yaw = 0.0
        self.obstacle_ahead = false;

        self.timer = self.create_timer(0.1, self.control_loop)
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
    
    def control_loop(self):
        x_distance = self.target_x - self.x
        y_distance = self.target_y - self.y

        distance = math.sqrt((x_distance) ** 2 + (y_distance) ** 2)
        desired_angle = math.atan2(y_distance, x_distance)
        
        angle_error = desired_angle - self.yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        twist = Twist()

        if (distance < 0.3):
            self.get_logger().info('Goal is reached!')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        elif (abs(angle_error) > 0.2):
            self.get_logger().info('Rotating to face the goal...')
            twist.linear.x = 0.0
            twist.angular.z = 0.5 * angle_error
        else:
            self.get_logger().info('Moving towards the goal...')
            twist.linear.x = 0.5 * distance
            twist.angular.z = 0.0


        
        


