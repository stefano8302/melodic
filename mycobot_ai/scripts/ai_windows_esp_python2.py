#!/usr/bin/env python
# encoding:utf-8

from Tkinter import ttk
from Tkinter import *
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
        self.win.wm_attributes('-topmost', 1)
        self.ros = False
        # archivo para ejecutar
        self.run_py = ""


        # establecer título
        self.win.title("Herramienta de inicio IAKit")
        self.win.geometry(
            "600x300+100+100")  # 290 160para el tamaño de la ventana，+10 +10 Defina la ubicación predeterminada cuando aparezca la ventana
        # abrir botón ros
        self.btn = Button(self.win, text="Abrir ROS", command=self.open_ros)
        self.btn.grid(row=0)

        self.chanse_code = Label(self.win, text="Elige un programa:", width=15)
        self.chanse_code.grid(row=1)

        self.myComboList = [u"reconocimiento de color", u"reconocimiento de objetos", u"Identificación de código QR"]
        self.myCombox = ttk.Combobox(self.win, values=self.myComboList)
        self.myCombox.grid(row=1, column=1)

        self.add_btn = Button(self.win, text="Agregar nueva imagen de objeto", command=self.add_img)
        self.add_btn.grid(row=1, column=2)

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
        self.tips = "1. Abrir ROS\n2. Seleccionar el programa que deseamos ejecutar y hacer clic en Ejecutar"

        self.btn = Button(self.win, text="Ejecutar", command=self.start_run)
        self.btn.grid(row=5)

        self.close = Button(self.win, text="Cerrar", command=self.close_py)
        self.close.grid(row=5, column=1)

        self.t2 = None
        self.log_data = Text(self.win, width=100, height=10)
        self.log_data.grid(row=16, column=0, columnspan=10)
        self.log_data.insert(END, self.tips)
        # self.code_list = ttk.Combobox(self.win, width=15)
        # self.code_list["value"] = ("reconocimiento de color", "reconocimiento de objetos", "Identificación de código QR")
        # self.code_list.current(0)
        # self.code_list.grid(row=1, column=1)

    def start_run(self):
        try:
            print(u"iniciar operación")
            one = self.myCombox.get()
            if one == u"reconocimiento de color":
                self.run_py = "detect_obj_color.py"
                t2 = threading.Thread(target=self.open_py1)
                t2.setDaemon(True)
                t2.start()
            elif one == u"reconocimiento de objetos":
                self.run_py = "detect_obj_img.py"
                t3 = threading.Thread(target=self.open_py)
                t3.setDaemon(True)
                t3.start()
            elif one == u"Identificación de código QR":
                self.run_py = "detect_encode.py"
                t3 = threading.Thread(target=self.open_py2)
                t3.setDaemon(True)
                t3.start()
        except Exception as e:
            self.tips = str(e)
            self.log_data.insert(END, self.tips)

    def open_py(self):
        os.system(
            #"python /home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_ai/scripts/detect_obj_img.py"
            "python /home/stefanos/catkin_ws/src/myCobotROS/mycobot_ai/scripts/detect_obj_img_fake.py"
        )

    def open_py1(self):
        os.system(
            #"python /home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_ai/scripts/detect_obj_color.py"
            "python /home/stefanos/catkin_ws/src/myCobotROS/mycobot_ai/scripts/detect_obj_color_fake.py"
        )

    def open_py2(self):
        os.system(
            #"python /home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_ai/scripts/detect_encode.py"
            "python /home/stefanos/catkin_ws/src/myCobotROS/mycobot_ai/scripts/detect_encode.py"
        )

    def add_img(self):
        os.system(
            #"python /home/ubuntu/catkin_ws/src/mycobot_ros/mycobot_ai/scripts/add_img.py"
            "python /home/stefanos/catkin_ws/src/myCobotROS/mycobot_ai/scripts/add_img.py"
        )

    def open_ros(self):
        if self.ros:
            print("ROS está arrancado")
            return
        t1 = threading.Thread(target=self.ross)
        t1.setDaemon(True)
        t1.start()
        self.ros = True

    def ross(self):
        os.system(
            #"roslaunch ~/catkin_ws/src/mycobot_ros/mycobot_ai/launch/vision.launch"
            "roslaunch /home/stefanos/catkin_ws/src/myCobotROS/mycobot_ai/launch/vision_fake.launch"
        )

    def close_py(self):
        t1 = threading.Thread(target=self.close_p)
        t1.setDaemon(True)
        t1.start()

    def close_p(self):
        # cierra el programa ai
        os.system("ps -ef | grep -E " + self.run_py +
                  " | grep -v 'grep' | awk '{print $2}' | xargs kill -9")

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
