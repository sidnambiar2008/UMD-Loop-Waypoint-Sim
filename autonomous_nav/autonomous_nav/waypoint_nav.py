import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from sensor_msgs.msg import LaserScan
import random

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, 'laser_scan', self.laser_callback, 10)


        # Starting odometry position
        self.x = 0.0
        self.y = 0.0

        # Random target positions
        self.waypoints = [(random.uniform(2.0, 6.0), random.uniform(-3.0, 3.0)) for _ in range(5)]
        self.current_wp_idx = 0
        self.target_x, self.target_y = self.waypoints[self.current_wp_idx]

        # Heading of the robot 
        self.yaw = 0.0
        self.obstacle_ahead = False
        self.timer = self.create_timer(0.1, self.control_loop)
        
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Gets the quaternion from the odometry message
        q = msg.pose.pose.orientation
        
        # Translate 3D aerospace math into a flat steering heading (Yaw in Euler Angles)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
    
    def control_loop(self):
        if self.current_wp_idx >= len(self.waypoints):
            self.get_logger().info('--- MISSION COMPLETION: All 5 Randomized Waypoint Goals Cleared! ---')
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.destroy_timer(self.timer) # Shuts down the thread cleanly to prevent runaway log prints
            return

        x_distance = self.target_x - self.x
        y_distance = self.target_y - self.y

        distance = math.sqrt((x_distance) ** 2 + (y_distance) ** 2)
        desired_angle = math.atan2(y_distance, x_distance)
        
        angle_error = desired_angle - self.yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        twist = Twist()

        # Immediate obstable check, spins away from the wall
        if self.obstacle_ahead:
            self.get_logger().info('Obstacle detected! ')
            twist.linear.x = 0.0
            twist.angular.z = 0.6

        elif (distance < 0.3):
            self.get_logger().info(f'Waypoint {self.current_wp_idx} reached! Moving to the next waypoint...')
            self.current_wp_idx += 1
            
            if self.current_wp_idx >= len(self.waypoints):
                self.get_logger().info('All sequential waypoints cleared!')
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                self.target_x, self.target_y = self.waypoints[self.current_wp_idx]
                

        # Adjusts Angle
        elif (abs(angle_error) > 0.2):
            self.get_logger().info('Rotating to face the goal...')
            twist.linear.x = 0.0
            twist.angular.z = 0.5 * angle_error

        # Moves towards the goal since angle is close and distance is not close
        else:
            self.get_logger().info('Moving towards the goal...')
            twist.linear.x = 0.5 * distance
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)
    
    def laser_callback(self, msg):
        front_rays = msg.ranges[70:110]

        if front_rays:
            min_distance = min(front_rays)

            # Ensures the obstacle centered in front of us is detected
            if min_distance < 1.5:
                self.obstacle_ahead = True
                self.get_logger().info('Obstacle detected ahead!.')
            else:
                self.obstacle_ahead = False


def main(args=None):
    # Initialize the ROS2 Communication Stack
    rclpy.init(args=args)

    # Create an instance of a node
    node = WaypointNavigator()

    # Spin the node so it stays alive and processes its 10Hz timer loops
    rclpy.spin(node)

    # Clean up memory and shutdown the ROS2 Communication Stack
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        
        


