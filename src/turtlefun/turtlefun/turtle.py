import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleDestination(Node):
    def __init__(self):
        super().__init__('turtle_destination_node')
        self.current_pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.current_pose_callback,
            10)
        self.desired_pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/destination',
            self.desired_pose_callback,
            10)
        
        self.current_pose = None
        self.desired_pose = None
        
        # Timer to get the turtle to the desired pose
        self.get_to_pose_timer = self.create_timer(0.1, self.get_to_pose)
        
        self.twist_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def current_pose_callback(self, msg):
        self.current_pose = msg
        
    def desired_pose_callback(self, msg):
        self.desired_pose = msg
        
    def get_to_pose(self):
        if not self.at_desired_position():
            # Orient the turtle towards the desired position
            if not self.is_facing_desired_position():
                self.turn_towards_position()
            else:
                self.move_towards_position()
        elif not self.is_facing_desired_theta():
            self.turn_towards_theta()
        else:
            # Stop the turtle
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.twist_publisher.publish(twist)
            self.get_logger().info('Turtle has reached the desired pose!')
        
    # Return true if the turtle is at the desired position or if no pose has been set
    def at_desired_position(self):
        if self.current_pose is None or self.desired_pose is None:
            return True
        
        return (abs(self.desired_pose.x - self.current_pose.x) < 0.1 and
                abs(self.desired_pose.y - self.current_pose.y) < 0.1)
    
    def is_facing_desired_position(self):
        if self.desired_pose is None or self.current_pose is None:
            return True
        destination_x = self.desired_pose.x - self.current_pose.x
        destination_y = self.desired_pose.y - self.current_pose.y
        # Angle of destination vector with respect to the x-axis
        angle_of_destination_vector = math.atan2(destination_y, destination_x)
        
        return abs(angle_of_destination_vector - self.current_pose.theta) < 0.1
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def turn_towards_position(self):
        destination_x = self.desired_pose.x - self.current_pose.x
        destination_y = self.desired_pose.y - self.current_pose.y
        # Angle of destination vector with respect to the x-axis
        angle_of_destination_vector = math.atan2(destination_y, destination_x)
        
        angle_of_rotation = angle_of_destination_vector - self.current_pose.theta
        angle_of_rotation = self.normalize_angle(angle_of_rotation)
        
        twist = Twist()
        twist.linear.x = 0.0
        # Turn towards the destination
        twist.angular.z = 2.0 * angle_of_rotation
        
        self.twist_publisher.publish(twist)
        
    def move_towards_position(self):
        destination_x = self.desired_pose.x - self.current_pose.x
        destination_y = self.desired_pose.y - self.current_pose.y
        distance_to_destination = math.sqrt(destination_x ** 2 + destination_y ** 2)
        
        twist = Twist()
        # Move towards the destination
        twist.linear.x = 0.5 * distance_to_destination
        twist.angular.z = 0.0
        self.twist_publisher.publish(twist)
        
    def is_facing_desired_theta(self):
        if self.desired_pose is None or self.current_pose is None:
            return True
        
        
        angle_of_rotation = self.desired_pose.theta - self.current_pose.theta
        angle_of_rotation = self.normalize_angle(angle_of_rotation)
        
        return abs(angle_of_rotation) < 0.1
         
    def turn_towards_theta(self):
        angle_of_rotation = self.desired_pose.theta - self.current_pose.theta
        angle_of_rotation = self.normalize_angle(angle_of_rotation)
        
        twist = Twist()
        twist.linear.x = 0.0
        # Turn towards the destination
        twist.angular.z = 2.0 * angle_of_rotation
        
        self.twist_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    turtle_destination = TurtleDestination()
    rclpy.spin(turtle_destination)

if __name__ == '__main__':
    main() 