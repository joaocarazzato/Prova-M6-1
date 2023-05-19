import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose as TPose
from time import sleep

# Definindo a diferença máxima aceita entre o objetivo e o que temos no momento
MAX_DIFF = 0.1

# Definindo a rota que utilizaremos
global posicoesrota 
posicoesrota = [
[0.0, 0.0],
[0.0, 0.5],
[0.5, 0.0],
[0.0, 0.5],
[0.0, 1.0],
[1.0, 0.0],
[0.0, 0.0]

]

# Mudando comportamentos default da classe pose do turtlesim
class Pose(TPose):

    def __init__(self, x=0.0, y=0.0, theta=0.0):
        super().__init__(x=x, y=y, theta=theta)

    def __repr__(self):
        return f"(x={self.x}, y={self.y})"
    
    def __add__(self, other):
        self.x += other.x
        self.y += other.y
        return self

    def __sub__(self, other):
        self.x -= other.x
        self.y -= other.y
        return self

    def __eq__(self, other):
        return abs(self.x - other.x) < MAX_DIFF and abs(self.y - other.y) < MAX_DIFF
    
# Classe do controlador é um nó do ROS
class TurtleController(Node):
    
    # Construtor da classe controladora.
    # Aqui cria-se o publisher, a subscription e o timer.
    # Como parâmetro é possível passar o período do controlador.
    def __init__(self, control_period=0.05):
        super().__init__('turtlecontroller')
        self.x_array = 1
        self.countqueue = 1
        self.countstack = 1
        self.pose = Pose(x = posicoesrota[(self.x_array - 1)][0], y = posicoesrota[(self.x_array - 1)][1])
        self.setpoint = Pose(x = posicoesrota[self.x_array][0], y = posicoesrota[self.x_array][1])
        print(f'self.pose: x={self.pose.x}, y={self.pose.y}')
        print(f'self.setpoint: x={self.setpoint.x}, y={self.setpoint.y}')

        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # O timer passa a ser um timer do loop de controle.
        # Não se deve iniciar loop de controle sem definir um período.
        self.control_timer = self.create_timer(timer_period_sec = control_period, callback = self.control_callback)


        # Método de callback da subscrição à pose da tartaruga.
        # Toda vez que chega uma mensagem no tópico de pose,
        # esse método é executado.
    def pose_callback(self, msg):
        self.pose = Pose(x=msg.x, y=msg.y, theta=msg.theta)
        if self.setpoint.x == posicoesrota[0][0] and self.setpoint.y == posicoesrota[0][1]:
            self.setpoint = Pose(x=(3.0 + self.pose.x)) # Se a pose for igual a inicial, definimos o setpoint com a pose + 3.0 de X
        self.get_logger().info(f"A tartaruga está em x={round(msg.x, 2)}, y={round(msg.y, 2)}, theta={round(msg.theta, 2)}")

        # Loop de controle. 
        # Aqui é onde define-se o acionamento dos motores do robô.
        # Roda a cada `control_period` segundos.
    def control_callback(self):
        if self.pose.x == posicoesrota[0][0] and self.pose.y == posicoesrota[0][1]:
            self.get_logger().info("Aguardando primeira pose...")
            return 
        msg = Twist()
        self.pose = Pose(x = posicoesrota[(self.x_array - 1)][0], y = posicoesrota[(self.x_array - 1)][1]) # Definindo a posicao como o x elemento da nossa rota
        self.setpoint = Pose(x = posicoesrota[self.x_array][0], y = posicoesrota[self.x_array][1]) # Definindo a próxima posicao como o x proximo elemento da rota

        x_diff = (self.pose.x + self.setpoint.x) - self.pose.x # Calculando as diferencas de X e Y para descobrirmos o quanto devemos andar
        y_diff = (self.pose.y + self.setpoint.y) - self.pose.y

        # Verificacao para transformar a fila em uma pilha quando chegar em seu limite, fazendo retorná-la a primeira posicao.
        if self.countqueue == (len(posicoesrota) - 1):
            if self.countqueue == (len(posicoesrota) - 1) and self.countstack == (len(posicoesrota) - 1):
                self.get_logger().info("Cheguei no meu destino.")
                exit()
            self.x_array = self.countqueue - self.countstack
            self.setpoint = Pose(x = -posicoesrota[self.x_array][0], y = -posicoesrota[self.x_array][1])
            x_diff = (self.pose.x + self.setpoint.x) - self.pose.x
            y_diff = (self.pose.y + self.setpoint.y) - self.pose.y
            self.countstack += 1
        else:
            self.countqueue += 1
            self.x_array = self.countqueue
        
        # Verificando se estão maiores ou menores que as diferencas aceitas(X e Y)
        if abs(x_diff) > MAX_DIFF:
            msg.linear.x = 1.0 if x_diff > 0 else -1.0
        else: msg.linear.x = 0.0
        if abs(y_diff) > MAX_DIFF:
            msg.linear.y = 1.0 if y_diff > 0 else -1.0
        else: msg.linear.y = 0.0
        self.publisher.publish(msg)
        print(f'self.pose: x={self.pose.x}, y={self.pose.y}')
        print(f'self.setpoint: x={self.setpoint.x}, y={self.setpoint.y}')
        print(f'x_diff= {x_diff}, y_diff= {y_diff}')
        print("self.x_array: ", self.x_array)
        sleep(2)

# Inicializo os nodo do ros e faco com que ele execute o nodo.
def main(args=None):
    rclpy.init(args=args)
    tc = TurtleController()
    rclpy.spin(tc)
    tc.destroy_node()
    rclpy.shutdown()

# Transformo em um script executavel.
if __name__ == "__main__":
    main()