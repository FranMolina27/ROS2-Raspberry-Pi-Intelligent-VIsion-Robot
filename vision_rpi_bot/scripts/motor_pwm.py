#Importando libreria para uso de los pines GPIO de la Raspberry
import RPi.GPIO as GPIO
import time
from time import sleep

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
pwm_motor_derecho = GPIO.PWM(motor_derecha_enable, 1000)
pwm_motor_izquierdo = GPIO.PWM(motor_izquierda_enable, 1000)

#Iniciando velocidad del motor (0 - 100)
pwm_motor_derecho.start(25)
pwm_motor_izquierdo.start(25)

def forward(seconds):
    print("Forward moving")
    GPIO.output(motor_derecha_cw, GPIO.HIGH)
    GPIO.output(motor_derecha_ccw, GPIO.LOW)
    GPIO.output(motor_izquierda_cw, GPIO.HIGH)
    GPIO.output(motor_izquierda_ccw, GPIO.LOW)
    sleep(seconds)

def reverse(seconds):
    print("Reverse moving")
    GPIO.output(motor_derecha_cw, GPIO.LOW)
    GPIO.output(motor_derecha_ccw, GPIO.HIGH)
    GPIO.output(motor_izquierda_cw, GPIO.LOW)
    GPIO.output(motor_izquierda_ccw, GPIO.HIGH)
    sleep(seconds)

def left(seconds):
    print("Left moving")
    GPIO.output(motor_derecha_cw, GPIO.LOW)
    GPIO.output(motor_derecha_ccw, GPIO.HIGH)
    GPIO.output(motor_izquierda_cw, GPIO.HIGH)
    GPIO.output(motor_izquierda_ccw, GPIO.LOW)
    sleep(seconds)

def right(seconds):
    print("Right moving")
    GPIO.output(motor_derecha_cw, GPIO.HIGH)
    GPIO.output(motor_derecha_ccw, GPIO.LOW)
    GPIO.output(motor_izquierda_cw, GPIO.LOW)
    GPIO.output(motor_izquierda_ccw, GPIO.HIGH)
    sleep(seconds)

def stop(seconds):
    print("Stopping motors")
    pwm_motor_derecho.ChangeDutyCycle(0)
    pwm_motor_izquierdo.ChangeDutyCycle(0)

def exit_():
    #Se deshabilitan los pines de la placa
    GPIO.cleanup()

def main():
    #Mover hacia adelante un tiempo t
    forward(2)
    reverse(2)
    left(2)
    right(2)
    stop()
    exit_()

#Función principal
if __name__ == '__main__':
    main()
