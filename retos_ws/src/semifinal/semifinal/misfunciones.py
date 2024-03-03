import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
import time
import math


def recto(pub, max_vel, distancia_total, acc_lin=0.005):
    # HECHO
    # TODO: Asegurarse que si la aceleracion es muy grande, no se pase de la velocidad maxima

    vel_linear = 0.01
    while(distancia_total >= 0):
        if (vel_linear < max_vel):
            vel_linear += acc_lin
            vel_linear += 0.005
        pub.publish_velocity((vel_linear, 0.0))
        time.sleep(0.1)
        distancia_recorrida = vel_linear*0.1
        distancia_total -= distancia_recorrida

    pub.publish_velocity((0.0, 0.0))


def atras(pub, max_vel, distancia_total, acc_lin=0.005):
    # HECHO
    
    vel_linear = 0.01
    while(distancia_total >= 0):
        if (vel_linear < max_vel):
            vel_linear += acc_lin
            vel_linear += 0.005
        pub.publish_velocity((-vel_linear, 0.0))
        time.sleep(0.1)
        distancia_recorrida = vel_linear*0.1
        distancia_total -= distancia_recorrida

    pub.publish_velocity((0.0, 0.0))

def derecha(pub, max_vel, degrees, acc_ang=0.01):
    #HECHO
    
    grados_total = degrees*math.pi/180
    
    vel_angular = 0.01
    while(grados_total >= 0):
        if (vel_angular < max_vel):
            vel_angular += acc_ang
            vel_angular += 0.01
        pub.publish_velocity((0.0, -vel_angular))
        time.sleep(0.1)
        grados_recorridos = vel_angular*0.1
        grados_total -= grados_recorridos

    pub.publish_velocity((0.0, 0.0))

def izquierda(pub, max_vel, degrees, acc_ang=0.01):
    # HECHO
    
    grados_total = degrees*math.pi/180
    
    vel_angular = 0.01
    while(grados_total >= 0):
        if (vel_angular < max_vel):
            vel_angular += acc_ang
            vel_angular += 0.01
        pub.publish_velocity((0.0, vel_angular))
        time.sleep(0.1)
        grados_recorridos = vel_angular*0.1
        grados_total -= grados_recorridos

    pub.publish_velocity((0.0, 0.0))
