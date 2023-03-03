import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
from time import sleep

from geometry_msgs.msg import Twist


class VelocitySubscriber(Node):

    def __init__(self):
        super().__init__('velocity_rpi_subscriber')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_to_pwm_callback, 10)
        #Asignación de pines para motor izquierdpo
        motor_izquierda_cw = 24
        motor_izquierda_ccw = 23
        motor_izquierda_enable = 25

        #Asignación de pines para motor derecho
        motor_derecha_cw = 14
        motor_derecha_ccw = 15
        motor_derecha_enable = 4

        #Definiendo modo de numeración de pines
        GPIO.setmode(GPIO.BCM)

        #Definiendo las acciones que se realizaran en los pines
        GPIO.setup(motor_derecha_cw, GPIO.OUT)
        GPIO.setup(motor_derecha_ccw, GPIO.OUT)
        GPIO.setup(motor_derecha_enable, GPIO.OUT)

        GPIO.setup(motor_izquierda_cw, GPIO.OUT)
        GPIO.setup(motor_izquierda_ccw, GPIO.OUT)
        GPIO.setup(motor_izquierda_enable, GPIO.OUT)

        #Definiendo pin de PWM
        self.pwm_motor_derecho = GPIO.PWM(motor_derecha_enable, 1000)
        self.pwm_motor_izquierdo = GPIO.PWM(motor_izquierda_enable, 1000)
        
        #Iniciando velocidad del motor (0 - 100)
        self.pwm_motor_derecho.start(25)
        self.pwm_motor_izquierdo.start(25)

        self.motor_derecha_a = motor_derecha_cw
        self.motor_derecha_b = motor_derecha_ccw
        self.motor_izquierda_a = motor_izquierda_cw
        self.motor_izquierda_b = motor_izquierda_ccw

    def cmd_to_pwm_callback(self, msg):
        llanta_derecha_vel = (msg.linear.x + msg.linear.z)/2
        llanta_izquierda_vel = (msg.linear.x - msg.linear.z)/2
        print(llanta_derecha_vel, "/", llanta_izquierda_vel)

        GPIO.output(self.motor_derecha_a, llanta_derecha_vel > 0)
        GPIO.output(self.motor_derecha_b, llanta_derecha_vel < 0)
        GPIO.output(self.motor_izquierda_a, llanta_izquierda_vel > 0)
        GPIO.output(self.motor_izquierda_b, llanta_izquierda_vel < 0)


def main(args=None):
    rclpy.init(args=args)

    velocity_subscriber = VelocitySubscriber()

    rclpy.spin(velocity_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    velocity_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()