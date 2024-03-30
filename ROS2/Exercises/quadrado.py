import rclpy
from rclpy.node import Node
from my_package.odom import Odom
from nav_msgs.msg import Odometry
import time
from geometry_msgs.msg import Twist
import numpy as np

class QuadradoNode(Node, Odom):

    def __init__(self):
        Node.__init__(self, 'quadrado_node')
        Odom.__init__(self)
        time.sleep(2)
        
        self.tempoInicio = self.get_clock().now()
        self.timer = self.create_timer(0.01, self.control)
        self.robot_state = 'andar'
        self.state_machine = {
            'andar': self.andar,
            'girar': self.girar
        }

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.yaw = 0.0 
        
    def andar(self):
        print('Andando')
        self.twist.linear.x = 0.4
        self.twist.angular.z = 0.0
        self.tempoAndando = self.get_clock().now()
        difference_in_seconds = (self.tempoAndando - self.tempoInicio).nanoseconds / 1e9
        if difference_in_seconds >= 2.0:
            self.robot_state = 'girar'
            self.goal_yaw = (self.yaw_2pi + np.pi/2) % (2 * np.pi)

    def girar(self):
        print('Girando')
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.2
        
        if np.abs(self.yaw_2pi - self.goal_yaw) <= np.deg2rad(2):
            self.robot_state = 'andar'
            self.tempoInicio = self.get_clock().now()

    def control(self):
        self.twist = Twist()
        self.state_machine[self.robot_state]()
        self.vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = QuadradoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()