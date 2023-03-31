#! /usr/bin/env python
# encoding: utf-8

import rospy
import time
from visualization_msgs.msg import Marker

class Send_marker(object):
    def __init__(self):
        # Heredar objeto de clase de objeto
        super(Send_marker, self).__init__()
        # Inicialice un nodo, si no se crea ningún nodo, la información no se puede publicar
        rospy.init_node("send_marker", anonymous=True)
        # Crear un publisher para publicar marcadores
        self.pub = rospy.Publisher("/cube", Marker, queue_size=1)
        # Crear un marcador para crear el modelo de bloques.
        self.marker = Marker()
        # Configure su propiedad y sus coordenadas son relativas a /base.
        # /base representa la parte inferior del brazo robótico en el modelo
        self.marker.header.frame_id = "/base"
        # Establecer el nombre del marcador
        self.marker.ns = "cube"
        # Establecer el tipo de marcador para bloquear
        self.marker.type = self.marker.CUBE
        # Establezca la acción del marcador, en este caso es agregar (agregue un marcador sin este nombre)
        self.marker.action = self.marker.ADD
        # Establezca el tamaño real del marcador, la unidad es metros
        self.marker.scale.x = 0.04
        self.marker.scale.y = 0.04
        self.marker.scale.z = 0.04
        # Establezca el color del marcador, 1.0 equivale a 255 (es necesario hacer una conversión proporcional)
        self.marker.color.a = 1.0
        self.marker.color.g = 0
        self.marker.color.r = 1.0
        # Inicializar la posición del marcador y su orientacion con los cuatro parametros
        self.marker.pose.position.x = 0
        self.marker.pose.position.y = 0
        self.marker.pose.position.z = 0.03
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0
        self.marker.pose.orientation.w = 1.0

    # Modificar las coordenadas y publicar el marcador
    def pub_marker(self, x, y, z=0.02):
        # Establecer la marca de tiempo del marcador
        self.marker.header.stamp = rospy.Time.now()
        # Establecer las coordenadas espaciales del marcador
        self.marker.pose.position.x = x
        self.marker.pose.position.y = y
        self.marker.pose.position.z = z
        # marcador de liberación
        #self.marker.color.g = self.color
        self.pub.publish(self.marker)

    # Deja que el marcador tenga un efecto de desplazamiento.
    def run(self):
        time.sleep(1)
        self.pub_marker(0.2, 0)
        

if __name__ == '__main__':
    marker = Send_marker()
    marker.run()
