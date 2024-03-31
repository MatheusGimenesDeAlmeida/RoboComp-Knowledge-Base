import rclpy
from rclpy.node import Node
# from my_package.laser import Laser
from entregavel_3.laser import Laser
from my_package.odom import Odom
from geometry_msgs.msg import Twist
from time import sleep
import numpy as np

class Limpador(Node, Laser, Odom): 

    def __init__(self):
        Node.__init__(self, 'limpador') 
        Laser.__init__(self)
        Odom.__init__(self)
        sleep(5)
        self.tempoInicio = self.get_clock().now()
        self.timer = self.create_timer(0.01, self.control)
        
        
        self.robot_state = 'forward'
        self.state_machine = {
            'forward': self.forward,
            'turn': self.turn
        }

        self.twist = Twist()
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def forward(self):
        self.twist.linear.x = 0.4
        self.twist.angular.z = 0.0
        
        frontDistance = min(self.front)
        if frontDistance < 0.5:
            self.goal_yaw = self.yaw_2pi + np.deg2rad(225)
            self.robot_state = 'turn'
            if self.goal_yaw >= 2*np.pi:
                self.goal_yaw -= 2*np.pi
           
    def turn(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.3
        print(self.yaw_2pi, self.goal_yaw)
        if abs(self.yaw_2pi - self.goal_yaw) < 0.05:
            self.robot_state = 'forward'
            print('forward')
        if self.yaw_2pi >= np.pi*2:
            self.yaw_2pi = 0
   

    def control(self):
        self.twist = Twist()
        # self.inf_right = self.laser_msg[225-self.openning:225+self.openning]
        self.state_machine[self.robot_state]()

        self.vel_pub.publish(self.twist)
       
           
def main(args=None):
    rclpy.init(args=args)
    ros_node = Limpador() 

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()