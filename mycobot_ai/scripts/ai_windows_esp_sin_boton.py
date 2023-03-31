#!/usr/bin/env python2
# encoding:utf-8


from Tkinter import *
import ttk
#from Tkinter.ttk import *

import os, time
import cv2
#from cv2 import cv2

import threading
from multiprocessing import Process

IS_CV_4 = cv2.__version__[0] == '4'
__version__ = "1.0"

print(cv2.__version__)

class Application(object):
    def __init__(self):
        self.win = Tk()
        # ventana en la parte superior
        self.win.wm_attributes('-topmost', 0)
        self.ros = False
        # archivo para ejecutar
        self.run_py = ""


        # establecer título
        self.win.title("Herramienta de Ejercicios y Simulaciones")
        self.win.geometry(
            "830x380+1150+10")  # 830 380 para el tamaño de la ventana，+1150 +10 Defina la ubicación predeterminada cuando aparezca la ventana
        
        # botón abrir roscore
        self.btn = Button(self.win, text="Lanzar roscore", command=self.roscore)
        self.btn.grid(row=0, column=3)

        # botón abrir nueva terminal
        self.btn = Button(self.win, text="Abrir nueva Terminal", command=self.terminal)
        self.btn.grid(row=1, column=3)

        self.chanse_code = Label(self.win, text="Elige un programa:", width=15)
        self.chanse_code.grid(row=0, column=0)

        # botón abrir nueva terminal
        self.btn = Button(self.win, text="RViz Deslizadores", command=self.rviz_desliza_fija)
        self.btn.grid(row=1, column=0)

        # botón abrir reconocimiento de color
        self.btn = Button(self.win, text="RViz MoveIt", command=self.rviz_moveit_fija)
        self.btn.grid(row=2, column=0)

        self.myComboList = [u"reconocimiento de color pinza", u"reconocimiento de color pinza MoveIt", u"reconocimiento de objetos pinza",
                            u"Identificación de código QR", u"Ventosa que coge cubo", u"Agregar imagen de objeto", u"Script Python con Moveit Commander",
                            u"Evitar Obstaculos", u"Moveit Commander Command Line", u"Robot Escribe LEIAL", u"Lector Codigo de Barras 1",
                            u"Lector Codigo de Barras 2", u"Brazo con Lector Codigo de Barras", u"Robot pick and place", u"Lite6 Robot Escribe LEIAL"]
        self.myCombox = ttk.Combobox(self.win, values=self.myComboList, width=31)
        self.myCombox.grid(row=0, column=2)
        
        self.myComboList2 = [u"RViz Deslizadores Pinza Fija", u"RViz Deslizadores Pinza Movil", u"RViz MoveIt Pinza Fija", u"RViz MoveIt Apagado Pinza Fija",
                             u"RViz MoveIt Pinza Movil", u"RViz+Gazebo Verter Cerveza", u"RViz Evitar Obstaculo", u"RViz+Gazebo 2 Robots y cinta",
                             u"RViz+Gazebo Ventosa coge cubo", u"RViz Escribir LEIAL", u"RViz Visión Artificial", u"RViz Visión Artificial Moveit", u"Lite6 RViz Moveit pinza fija"]
        self.myCombox2 = ttk.Combobox(self.win, values=self.myComboList2, width=28)
        self.myCombox2.grid(row=0, column=1)

        #self.add_btn = Button(self.win, text="Agregar imagen de objeto", command=self.add_img)
        #self.add_btn.grid(row=1, column=2)

        # self.set_xy = Label(self.win, text="set_xy:", width=10)
        # self.set_xy.grid(row=1)

        # self.x = Label(self.win, text="x:")
        # self.x.grid(row=2)
        # self.v1 = StringVar()
        # self.e1 = Entry(self.win,textvariable=self.v1, width=10)
        # self.e1.insert(0,0)
        # self.e1.grid(row=2,column=1)

        # self.y = Label(self.win, text="y:")
        # self.y.grid(row=3)
        # self.v2 = StringVar()
        # self.e2 = Entry(self.win,textvariable=self.v2, width=10)
        # self.e2.insert(0,0)
        # self.e2.grid(row=3,column=1)
        self.tips = "1. Seleccionar antes en el primer desplegable a la izquierda el programa de tipo Launch que se desea ejecutar\n2. Hacer clic en el botón Ejecutar correspondiente\n3. Una vez terminado de cargar, seleccionar en el segundo desplegable uno de los scripts Python que deseamos ejecutar\n4. Hacer clic en el botón Ejecutar correspondiente\n5. Antes de ejecutar un nuevo programa de tipo 'Launch', selecionar la ventana terminal donde se está ejecutando este programa y presionar a la vez las teclas 'Ctrl' y 'c' para cerrar el programa launch'\n6. En la mayoría de los casos no es necesario cerrar manualmente los scripts de Python pero, si lo fuera, el procedimiento es siempre presionar a la vez las teclas 'Ctrl' y 'c'\n7. Para cerrar este programa, antes es necesario cerrar el programa de tipo Launch y luego repetir nuevamente el presionar a la vez las teclas 'Ctrl' y 'c'. La ventana de la GUI se cerrará en cuanto mováis el cursor encima de la misma."

        self.btn = Button(self.win, text="Ejecutar Python", command=self.start_run)
        self.btn.grid(row=1, column=2)
        
        self.btn = Button(self.win, text="Ejecutar Launch", command=self.start_run2)
        self.btn.grid(row=1, column=1)

        #self.close = Button(self.win, text="Cerrar Py", command=self.close_p)
        #self.close.grid(row=2, column=2)

        #self.close = Button(self.win, text="Cerrar Launch", command=self.close_p2)
        #self.close.grid(row=2, column=1)

        self.t2 = None
        self.log_data = Text(self.win, width=100, height=15) #tamaño de la celda grande tips
        self.log_data.grid(row=16, column=0, columnspan=10)
        self.log_data.insert(END, self.tips)
        # self.code_list = ttk.Combobox(self.win, width=15)
        # self.code_list["value"] = ("reconocimiento de color", "reconocimiento de objetos", "Identificación de código QR")
        # self.code_list.current(0)
        # self.code_list.grid(row=1, column=1)

    def start_run(self):
        try:
            print(u"iniciar operación Python")
            one = self.myCombox.get()
            if one == u"reconocimiento de color pinza":
                self.run_py = "detect_obj_color_pinza.py"
                t1 = threading.Thread(target=self.open_py1)
                t1.setDaemon(True)
                t1.start()
            elif one == u"reconocimiento de objetos":
                self.run_py = "detect_obj_img_pinza.py"
                t1 = threading.Thread(target=self.open_py)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Identificación de código QR":
                self.run_py = "detect_encode.py"
                t1 = threading.Thread(target=self.open_py2)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Ventosa que coge cubo":
                self.run_py = "mycobot320_legrip_pi_pl.py"
                t1 = threading.Thread(target=self.open_pysuction)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Agregar imagen de objeto":
                self.run_py = "add_img.py"
                t1 = threading.Thread(target=self.add_img)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Script Python con Moveit Commander":
                self.run_py = "mycobot320_legrip_pi_pl2.py"
                t1 = threading.Thread(target=self.python_moveitcommander)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Evitar Obstaculos":
                self.run_py = "mycobot320_avoid.py"
                t1 = threading.Thread(target=self.python_avoid)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Moveit Commander Command Line":
                self.run_py = "moveit_commander_cmdline.py"
                t1 = threading.Thread(target=self.python_moveitcommander_cmdline)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Robot Escribe LEIAL":
                self.run_py = "mycobot320_writing.py"
                t1 = threading.Thread(target=self.python_writing)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Lector Codigo de Barras 1":
                self.run_py = "barcodereader1.py"
                t1 = threading.Thread(target=self.python_barcodereader1)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Lector Codigo de Barras 2":
                self.run_py = "barcodereader2.py"
                t1 = threading.Thread(target=self.python_barcodereader2)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Brazo con Lector Codigo de Barras":
                self.run_py = "mycobot320_barcodereader.py"
                t1 = threading.Thread(target=self.python_brazo_barcodereader)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Robot pick and place":
                self.run_py = "mycobot320_pick_place.py"
                t1 = threading.Thread(target=self.python_picknplace)
                t1.setDaemon(True)
                t1.start()
            elif one == u"reconocimiento de color pinza MoveIt":
                self.run_py = "detect_obj_color_moveit.py"
                t1 = threading.Thread(target=self.python_py1_moveit)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Lite6 Robot Escribe LEIAL":
                self.run_py = "lite6_writing.py"
                t1 = threading.Thread(target=self.lite6_python_writing)
                t1.setDaemon(True)
                t1.start()


        except Exception as e:
            self.tips = str(e)
            self.log_data.insert(END, self.tips)
                    
    def start_run2(self):
        try:
            print(u"iniciar operación Launch")
            one = self.myCombox2.get()
            if one == u"RViz Deslizadores Pinza Fija":
                self.run_py = "mycobot_320_slider.launch"
                t1 = threading.Thread(target=self.rviz_desliza_fija)
                t1.setDaemon(True)
                t1.start()
            elif one == u"RViz Deslizadores Pinza Movil":
                self.run_py = "mycobot_320_slider_gripper.launch"
                t1 = threading.Thread(target=self.rviz_desliza_movil)
                t1.setDaemon(True)
                t1.start()
            elif one == u"RViz MoveIt Pinza Fija":
                self.run_py = "demo.launch"
                t1 = threading.Thread(target=self.rviz_moveit_fija)
                t1.setDaemon(True)
                t1.start()
            elif one == u"RViz MoveIt Apagado Pinza Fija":
                self.run_py = "demo_moveit_apagado.launch"
                t1 = threading.Thread(target=self.rviz_moveit_apagado_fija)
                t1.setDaemon(True)
                t1.start()
            elif one == u"RViz MoveIt Pinza Movil":
                self.run_py = "demo_pinza.launch"
                t1 = threading.Thread(target=self.rviz_moveit_movil)
                t1.setDaemon(True)
                t1.start()
            elif one == u"RViz+Gazebo Verter Cerveza":
                self.run_py = "demo_gazebo_pouring.launch"
                t1 = threading.Thread(target=self.simu_cerveza)
                t1.setDaemon(True)
                t1.start()
            elif one == u"RViz Evitar Obstaculo":
                self.run_py = "demo_avoid.launch"
                t1 = threading.Thread(target=self.rviz_avoid)
                t1.setDaemon(True)
                t1.start()
            elif one == u"RViz+Gazebo 2 Robots y cinta":
                self.run_py = "demo_robots.launch"
                t1 = threading.Thread(target=self.simu_2robocinta)
                t1.setDaemon(True)
                t1.start()
            elif one == u"RViz+Gazebo Ventosa coge cubo":
                self.run_py = "demo_gazebo_suctiongrip.launch"
                t1 = threading.Thread(target=self.simu_sucticube)
                t1.setDaemon(True)
                t1.start()
            elif one == u"RViz Escribir LEIAL":
                self.run_py = "demo_writing.launch"
                t1 = threading.Thread(target=self.rviz_writing)
                t1.setDaemon(True)
                t1.start()
            elif one == u"RViz Visión Artificial":
                self.run_py = "vision_fake.launch"
                t1 = threading.Thread(target=self.vis_art)
                t1.setDaemon(True)
                t1.start()
            elif one == u"RViz Visión Artificial Moveit":
                self.run_py = "demo_vision_moveit_apagado.launch"
                t1 = threading.Thread(target=self.vis_art_moveit)
                t1.setDaemon(True)
                t1.start()
            elif one == u"Lite6 RViz Moveit pinza fija":
                self.run_py = "demo.launch"
                t1 = threading.Thread(target=self.lite6_rviz_moveit_fija)
                t1.setDaemon(True)
                t1.start()
        except Exception as e:
            self.tips = str(e)
            self.log_data.insert(END, self.tips)

    
    def open_py(self):
        os.system(
            #"python /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_ai/scripts/detect_obj_img.py"
            "python /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_ai/scripts/detect_obj_img_pinza.py"
        )

    def open_py1(self):
        os.system(
            #"python /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_ai/scripts/detect_obj_color.py"
            "python /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_ai/scripts/detect_obj_color_pinza.py"
        )
    def python_py1_moveit(self):
        os.system(
            "python /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_ai/scripts/detect_obj_color_pinza_moveit.py"
        )
    def open_py2(self):
        os.system(
            #"python /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_ai/scripts/detect_encode.py"
            "python /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_ai/scripts/detect_encode.py"
        )

    def add_img(self):
        os.system(
            #"python /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_ai/scripts/add_img.py"
            "python /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_ai/scripts/add_img.py"
        )
        
    def open_pysuction(self):
        os.system(
            "python /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_320_moveit/scripts/mycobot320_legrip_pi_pl.py"
        )

    def python_moveitcommander(self):
        os.system(
            "python /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_320_moveit/scripts/mycobot320_legrip_pi_pl2.py"
        )

    def python_avoid(self):
        os.system(
            "python /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_320_moveit/scripts/mycobot320_avoid.py"
        )

    def python_moveitcommander_cmdline(self):
        os.system(
            "rosrun moveit_commander moveit_commander_cmdline.py"
        )

    def python_writing(self):
        os.system(
            "python /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_320_moveit/scripts/mycobot320_writing.py"
        )
    
    def python_barcodereader1(self):
        os.system(
            "python /home/stefanos/catkin_ws/src/mycobot_ros/lector_codigo_barras/barcodereader1.py"
        )

    def python_barcodereader2(self):
        os.system(
            "python /home/stefanos/catkin_ws/src/mycobot_ros/lector_codigo_barras/barcodereader2.py"
        )

    def python_brazo_barcodereader(self):
        os.system(
            "python /home/stefanos/catkin_ws/src/mycobot_ros/lector_codigo_barras/mycobot320_barcodereader.py"
        )

    def python_picknplace(self):
        os.system(
            "python /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_320_moveit/scripts/mycobot320_pick_place.py"
        )

    def lite6_python_writing(self):
        os.system(
            "python /home/stefanos/catkin_ws/src/xarm_ros/lite6_moveit_config/scripts/lite6_writing.py"
        )

    def roscore(self):
        os.system(
            "roscore"
        )
        
    def terminal(self):
        os.system(
            "gnome-terminal"
        )

    def vis_art(self):
        os.system(
            #"roslaunch ~/catkin_ws/src/mycobot_ros/mycobot_ai/launch/vision.launch"
            "roslaunch /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_ai/launch/vision_320.launch"
        )

    def vis_art_moveit(self):
        os.system(
            #"roslaunch ~/catkin_ws/src/mycobot_ros/mycobot_ai/launch/vision.launch"
            "roslaunch /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_320_moveit/launch/demo_vision_moveit_apagado.launch"
        )

    def rviz_desliza_fija(self):
        os.system(
            "roslaunch /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_320/launch/mycobot_320_slider.launch"
        )
    
    def rviz_desliza_movil(self):
        os.system(
            "roslaunch /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_320/launch/mycobot_320_slider_gripper.launch"
        )
    
    def rviz_moveit_fija(self):
        os.system(
            "roslaunch /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_320_moveit/launch/demo.launch"
        )
    def rviz_moveit_apagado_fija(self):
        os.system(
            "roslaunch /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_320_moveit/launch/demo_moveit_apagado.launch"
        )
    def rviz_moveit_movil(self):
        os.system(
            "roslaunch /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_320_moveit/launch/demo_pinza.launch"
        )
    def lite6_rviz_moveit_fija(self):
        os.system(
            "roslaunch /home/stefanos/catkin_ws/src/xarm_ros/lite6_moveit_config/launch/demo.launch"
        )
            
    def simu_cerveza(self):
        os.system(
            "roslaunch /home/stefanos/catkin_ws/src/mycobot_ros/mycobot_320_moveit_pi_pl/launch/demo_gazebo_pouring.launch"
        )
        
    #def rviz_avoid(self):
    #    os.system(
    #        "roslaunch /home/stefanos/catkin_ws/src/myCobotROS/mycobot_320_moveit_pi_pl/launch/demo_avoid.launch"
    #    )
    
    #def simu_2robocinta(self):
    #    os.system(
    #        "roslaunch /home/stefanos/catkin_ws/src/conveyor_demo/src/demo_world/launch/demo_robots.launch"
    #    )
        
    #def simu_sucticube(self):
    #    os.system(
    #        "roslaunch /home/stefanos/catkin_ws/src/myCobotROS/mycobot_320_moveit_pi_pl/launch/demo_gazebo_suctiongrip.launch"
    #    )
            
    def rviz_writing(self):
        os.system(
            "roslaunch /home/stefanos/catkin_ws/src/myCobotROS/mycobot_320_moveit_pi_pl/launch/demo_writing.launch"
        )
        
    #def close_py(self):
    #    t1 = threading.Thread(target=self.close_p)
    #    t1.setDaemon(True)
    #    t1.start()
        
    #def close_py2(self):
    #    t1 = threading.Thread(target=self.close_p2)
    #    t1.setDaemon(True)
    #    t1.start()

    #def close_p(self):
        # cierra el programa ai
    #    os.system("ps -ef | grep -E " + self.run_py +
    #              " | grep -v 'grep' | awk '{print $2}' | xargs kill -9")
        # "ps -ef" saca una lista de los procesos; "grep -v 'grep'" muestra todas las lineas que no corresponden al filtro puesto; "awk '{print $2}'" imprime la palabra que se encuentra en el segundo campo; "xargs kill -9" compila y ejecuta lineas de comando, en este caso kill
    #def close_p2(self):
        # cierra el programa
    #    os.system("ps -ef | grep -E " + self.run_py +
    #              " | grep -v 'grep' | awk '{print $2}' | xargs kill -9 | killall -9 rviz gazebo roscore rosmaster gazebo_controller_spawner controller_spawner controller_manager rosout gazebo_gui spawn_gazebo_model joint_state_publisher robot_state_publisher move_group")
                   
    def get_current_time(self):
        # tiempo de registro
        """Obtener la hora actual con formato."""
        current_time = time.strftime("%Y-%m-%d %H:%M:%S",
                                     time.localtime(time.time()))
        return current_time

    def write_log_to_Text(self, logmsg):
        # establecer la función de registro
        global LOG_NUM
        current_time = self.get_current_time()
        logmsg_in = str(current_time) + " " + str(logmsg) + "\n"  # nueva línea

        if LOG_NUM <= 18:
            self.log_data_Text.insert(END, logmsg_in)
            LOG_NUM += len(logmsg_in.split("\n"))
            # print(LOG_NUM)
        else:
            self.log_data_Text.insert(END, logmsg_in)
            self.log_data_Text.yview("fin")

    def run(self):
        self.win.mainloop()


if __name__ == "__main__":
    mc = Application()
    mc.run()
