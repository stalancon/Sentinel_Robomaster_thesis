
import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool


class AnomalyPublisher(Node):

    def __init__(self):
        super().__init__('anomaly_publisher')
        self.publisher_ = self.create_publisher(Bool, '/anomaly', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Bool()
  
        if self.i == 33:
            msg.data = True        

            self.publisher_.publish(msg)
                     	
        self.i += 1

    def stop(self):
    	self.get_logger().info('Finished ...')

def main(args=None):
    rclpy.init(args=args)
    node = AnomalyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()

