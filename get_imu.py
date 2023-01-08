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
data_power = {
"Vcc":0,
"Vservo":0,
"flags":0
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
                
            elif msg.get_type() == 'POWER_STATUS':
                pdata = str(msg)
                try:
                    pdata = pdata.split("{")
                    pdata = pdata[1].strip("}").split(",")
                    #print(pdata)
                    for i in pdata:
                        data_power[i.split(":")[0].strip()] = np.float64(i.split(":")[1].strip()) / 1000

                    #data_power["Vcc"] = np.float64(pdata[0].split(":")[1].strip())/1000
                    #data_power["Vservo"] = np.float64(pdata[0].split(":")[1].strip())/1000
                    #print(data_power)
                    #print(type(data_imu["pitch"]))
                except:
                    print(data)
            elif msg.get_type() == 'GPS_RAW_INT':
                print(str(msg))
                try:
                    pass
                except:
                    print(data)
            else:
                pass
                #print(msg.get_type(), msg)

get_imu = Thread(target=get_data)
get_imu.start()
"""
while 1:
    pitch = np.round(data_imu["pitch"]*100,2)
    if pitch != 0:
        print(pitch)"""