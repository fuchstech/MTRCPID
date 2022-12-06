from PyQt5 import QtGui
from PyQt5.QtWidgets import QWidget, QApplication, QLabel, QVBoxLayout
from PyQt5.QtGui import QPixmap
import sys
import cv2
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
import numpy as np
from pymavlink import mavutil
from threading import Thread
from time import sleep
from serial import SerialException
global master
try:
    master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
    master.wait_heartbeat()
except SerialException:
    print("Pixhawk is not connected")
    sys.exit()
connection = True
print("Connected")
data_imu = {
"roll":0,
"pitch":0,
"yaw":0,
"altitude":0,}

def get_data():
    if connection:
        while True:
            msg = master.recv_match()
            if not msg:
                continue
            #print(msg.get_type())

            if msg.get_type() == 'AHRS2':
                data = str(msg)
                try:
                    data = data.split(":")
                    data_imu["roll"] = np.float64(data[1].split(",")[0])
                    data_imu["pitch"] = np.float64(data[2].split(",")[0])
                    data_imu["yaw"] = np.float64(data[3].split(",")[0])
                    data_imu["altitude"] = np.float64(data[4].split(",")[0])
                    #print(type(data_imu["pitch"]))
                except:
                    print(data)
            else:
                pass
                #print(msg.get_type(), msg)

get_imu = Thread(target=get_data)
get_imu.start()

foreground = cv2.imread('tilki.png')

# position of logo on video
top_left = [0, 0] # the top-left corner of your logo goes here
tx = top_left[0] # less typing later
ty = top_left[1]

# crop of logo
left = 0
right = 100
top = 0 # y = 0 is the top of the image
bottom = 100

# calculate width and height of logo crop
width = right - left
height = bottom - top

# main loop
alpha = 0.4

#python3.8 -m pip install opencv-python==4.3.0.36
class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def run(self):
        # capture from web cam
        cap = cv2.VideoCapture(0)
        posX = 5
        X1 = 100
        X2 = 540
        color = (39,237,250)
        while True:
            
            ret, cv_img = cap.read()
            cv_slice = cv_img[ty:ty+height, tx:tx+width]; 
            logo_slice = foreground[top:bottom, left:right]; 
            added_image = cv2.addWeighted(cv_slice,alpha,logo_slice,1-alpha,0)
            cv_img[ty:ty+height, tx:tx+width] = added_image
            #cv2.line(cv_img, (40,posX), (600,posX),(255,255,255),1)
            cv2.line(cv_img, (X1,30),(X1,450),color,2)
            cv2.line(cv_img, (X2,30),(X2,450),color,2)
            cv2.line(cv_img, (540,posX+240), (640,posX+240),color,1)
            cv2.line(cv_img, (0,posX+240), (100,posX+240),color,1)
            times = 25
            for i in range(17):
                if i%3 == 1:

                    cv2.line(cv_img, (X1,posX+i*times), (X1+30,posX+i*times),color,3)
                    cv2.line(cv_img, (X2,posX+i*times), (X2-30,posX+i*times),color,3)
                else:
                    cv2.line(cv_img, (X1,posX+i*times), (X1+20,posX+i*times),color,2)
                    cv2.line(cv_img, (X2,posX+i*times), (X2-20,posX+i*times),color,2) 
                
            if ret:
                pitch = np.round(data_imu["pitch"]*100*2,1)
                self.change_pixmap_signal.emit(cv_img)
                posX = 0 + np.int64(pitch)

                print(posX,pitch,np.int64(pitch))



class App(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Qt live label demo")
        self.disply_width = 640
        self.display_height = 480
        # create the label that holds the image
        self.image_label = QLabel(self)
        self.image_label.resize(self.disply_width, self.display_height)
        # create a text label
        self.textLabel = QLabel('Webcam')
        # create a vertical box layout and add the two labels
        vbox = QVBoxLayout()
        vbox.addWidget(self.image_label)
        vbox.addWidget(self.textLabel)
        # set the vbox layout as the widgets layout
        self.setLayout(vbox)
#-------------------
        # create the video capture thread
        self.thread = VideoThread()
        # connect its signal to the update_image slot
        self.thread.change_pixmap_signal.connect(self.update_image)
        # start the thread
        self.thread.start()



    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """Updates the image_label with a new opencv image"""
        qt_img = self.convert_cv_qt(cv_img)
        self.image_label.setPixmap(qt_img)
    
    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.disply_width, self.display_height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)
    
if __name__=="__main__":
    app = QApplication(sys.argv)
    a = App()
    a.show()
    sys.exit(app.exec_())
