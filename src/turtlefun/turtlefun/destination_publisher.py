import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class DestinationPublisher(Node):
    def __init__(self):
        super().__init__('destination_publisher_node')
        self.publisher = self.create_publisher(Pose, '/turtle1/destination', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pose = Pose()
        self.pose.x = 2.0
        self.pose.y = 10.0
        self.pose.theta = -2.9

    def timer_callback(self):
        self.publisher.publish(self.pose)
        self.get_logger().info('Publishing: "%s"' % self.pose)
        
def main(args=None):
    rclpy.init(args=args)
    destination_publisher = DestinationPublisher()
    rclpy.spin_once(destination_publisher)
    destination_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()