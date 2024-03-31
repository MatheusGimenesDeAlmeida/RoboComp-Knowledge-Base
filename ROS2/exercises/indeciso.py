import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from my_package.laser import Laser
import numpy as np
import time

class IndecisoNode(Node, Laser):

    def __init__(self):
        Node.__init__(self, 'indeciso_node')
        Laser.__init__(self)
        time.sleep(2)
        
        self.tempoInicio = self.get_clock().now()
        self.timer = self.create_timer(0.2, self.control)

        self.robot_state = 'forward'
        self.state_machine = {
            'forward': self.forward,
            'backward': self.backward,
            'stop': self.stop,
        }

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist
        
    def forward(self):
        print('Andando')
        self.twist.linear.x = 0.4
        
        
    def backward(self):
        print('ré')
        self.twist.linear.x = - 0.4

    def stop(self):
        print('parado')
        self.twist.linear.x = 0.0

        
    def control(self):
        self.twist = Twist()
        self.state_machine[self.robot_state]()
        self.vel_pub.publish(self.twist)
        if min(self.front) <= 0.95:
            self.robot_state = 'backward'
        elif min(self.front) >= 1.05:
            self.robot_state = 'forward'
        else :
            self.robot_state = "stop"

def main(args=None):
    rclpy.init(args=args)
    node = IndecisoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()