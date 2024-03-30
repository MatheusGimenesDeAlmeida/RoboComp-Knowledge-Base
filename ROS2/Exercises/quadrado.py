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
        
        self.tempoInicio = self.get_clock().now() # Pega o tempo quando começou a rodar o código
        self.timer = self.create_timer(0.01, self.control) # Cria um timer para o programa
        self.robot_state = 'andar'     # Define o estado inicial do robô
        self.state_machine = {         # Cria umdicionário de estados 
            'andar': self.andar,
            'girar': self.girar
        }

        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)  # Cria um Publisher com mensagens do tipo Twist, sobre o tópico cdm_vel, com tamanho de fila igual a 10
        self.yaw = 0.0 # Deixa o objeto alinhado e no eixo de origem 
        
    def andar(self):
        print('Andando')
        self.twist.linear.x = 0.4
        self.twist.angular.z = 0.0
        self.tempoAndando = self.get_clock().now()   # Retorna o tempo atual 
        difference_in_seconds = (self.tempoAndando - self.tempoInicio).nanoseconds / 1e9  # Tempo em que o robô está andando 
        if difference_in_seconds >= 2.0: # Quando o robô estiver andando a mais de 2 segundos ele muda seu estado para girar
            self.robot_state = 'girar'
            self.goal_yaw = (self.yaw_2pi + np.pi/2) % (2 * np.pi) # Self.goal_yaw é o ângulo desejado do carrinho, manda o robô girar 90 graus anti-horário
            # np.pi/2 é o ângulo que o robô vai girar

    def girar(self):
        print('Girando')
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.2
        
        if np.abs(self.yaw_2pi - self.goal_yaw) <= np.deg2rad(2): # Cas o a diferença do ângulo inicial e 
            self.robot_state = 'andar'
            self.tempoInicio = self.get_clock().now() # Reinicia o relógio para quando voltar para o estado andando, ande 2 segundos 

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