import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
import numpy as np
from my_package.odom import Odom
from my_package.laser import Laser

# Adicione aqui os imports necessários

class Explorador(Node, Odom, Laser): # Mude o nome da classe

    def __init__(self):
        Node.__init__(self, 'explorador_node') # Mude o nome do nó
        Odom.__init__(self) # Mude o nome do nó
        Laser.__init__(self) # Mude o nome do nó

        self.timer = self.create_timer(0.1, self.control)
        self.openning = 15

        self.robot_state = 'girar_horario' # Estado inicial do robô
        self.state_machine = { # Estados disponíveis 
            'stop': self.stop, # Parado
            'girar': self.girar, # Girando
            'andar':self.andar, # Andando
            'girar_horario':self.girar_horario # Girando horário
        }

        # Inicialização de variáveis
        self.twist = Twist()
        
        # Subscribers
        ## Coloque aqui os subscribers

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        ## Coloque aqui os publishers

        self.goal_girar = False
        self.hora_de_sair = False # Atributos booleano que servem para controlar o robô
        self.vamos_embora = False # Atributos booleano que servem para controlar o robô
        self.goal_andar = False
        self.contador = 0


    def stop(self):
        self.twist = Twist()

    def girar(self):
        if not self.goal_girar:
            self.goal_girar = (self.yaw_2pi + np.pi / 2) % (2 * np.pi) # Código para girar 90 graus anti-horário
    
        self.twist.angular.z = 0.1
        
        diferenca_yaw = (self.goal_girar - self.yaw_2pi) % (2 * np.pi)
        print('diferenca_yaw: ', np.degrees(diferenca_yaw))
        if diferenca_yaw <= np.radians(2):
            self.goal_girar = False  
            self.twist.angular.z = 0.0
            self.contador += 1
            self.robot_state = 'andar'

    def girar_horario(self): #Estado inicial do robô, depois ele só gira anti-horário
        if not self.goal_girar: # self.goal_girar tem que ser false (ele começa false)
            self.goal_girar = (self.yaw_2pi - np.pi / 2) % (2 * np.pi) # self.goal_girar guarda o ângulo desejado 
            # Mesmo código do quadrado (APS) só que - np.pi/2, pois agora é pra girar para o outro lado, então a angulação final em comparação à inicial vai ser diferente
        
        self.twist.angular.z = -0.1 # Velocidade angular como negativa, pois está anti horário 
        
        diferenca_yaw = (self.goal_girar - self.yaw_2pi) % (2 * np.pi) # Aqui guarda adiferença do angulo guardado desejado e do angulo atual
        print('diferenca_yaw: ', np.degrees(diferenca_yaw))
        if diferenca_yaw <= np.radians(2): # Quando a diferença do angulo desejado e do angulo atual for pequena:
            self.goal_girar = False  # O angulo desejado para girar se torna falso
            self.contador += 1
            self.twist.angular.z = 0.0 # velocidade angular vira 0, ou seja ele para de girar
            self.robot_state = 'andar' # O robô começa a andar
            # Portanto, o robô só começa a andar quando seu ângulo (self.yaw_2pi) chegou muito próximo do objetivo (self.goal_girar)
    
    def andar(self):
        self.twist.linear.x = 0.5
        print(self.contador)
        print(np.min(self.front)) # Printa o valor mínimo da lista de distancia frontal do robô

        if self.vamos_embora == True and (np.max(self.left) == np.inf and np.max(self.right) == np.inf) and self.counter == 6:
            self.twist.linear.x = 0.0
            self.robot_stat = 'stop'
        elif np.min(self.front) < 0.5 and self.contador < 5:
            self.robot_state = 'girar'
        elif np.min(self.right) == np.inf and self.contador == 5:
            self.vamos_embora = True
            self.robot_state = 'girar_horario'

    def control(self):
        self.twist = Twist()
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()

        self.cmd_vel_pub.publish(self.twist)
        
            
def main(args=None):
    rclpy.init(args=args)
    ros_node = Explorador() # Mude o nome da classe

    rclpy.spin(ros_node)

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()