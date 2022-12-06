from pymavlink import mavutil
from threading import Thread
from time import sleep
import numpy as np

master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master.wait_heartbeat()
connection = True
print("Connected")
data_imu = {
"roll":0,
"pitch":0,
"yaw":0,
"altitude":0,
}

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

while 1:
    pitch = np.round(data_imu["pitch"]*100,2)
    if pitch != 0:
        print(pitch)