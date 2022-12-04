from pymavlink import mavutil
from threading import Thread
from time import sleep
from simple_pid import PID
#master = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
master.wait_heartbeat()
print("Connected")
data_imu = {"ygyro":0}
def get_data(show=False):
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        #print(msg.get_type())

        if msg.get_type() == 'RAW_IMU':
            data = str(msg)
            try:
                data = data.split(":")
                data_imu["xacc"]= data[2].split(",")[0]
                data_imu["yacc"] = data[3].split(",")[0]
                data_imu["zacc"] = data[4].split(",")[0]
                data_imu["xgyro"]= data[5].split(",")[0]
                data_imu["ygyro"] = data[6].split(",")[0]
                data_imu["zgyro"] = data[7].split(",")[0]
                data_imu["xmag"] = data[8].split(",")[0]
                data_imu["ygyro"] = data[9].split(",")[0]
                data_imu["zgyro"] = data[10].split(",")[0][0:-1]
            except:
                print(data)
        else:
            print(msg.get_type())

def arm():
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

    # wait until arming confirmed (can manually check with master.motors_armed())
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('Armed!')
def disarm():
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

    # wait until disarming confirmed
    master.motors_disarmed_wait()
    print("disarm")
def set_rc_channel_pwm(id, pwm=1500):
    
    if id < 1:
        print("Channel does not exist.")
        return

    #http://mavlink.org/messages/common#RC_CHANNELS_OVERRIDE
    if id < 9:
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id - 1] = pwm
        master.mav.rc_channels_override_send(
            master.target_system, 
            master.target_component,             # target_component
            *rc_channel_values)                  # Rc channel listesi 

arm()
get = Thread(target=get_data)
get.start()
print("initializing")
sleep(5)

while True:
    try:
        
        y = int(data_imu["ygyro"])
        if y*-8>900 and y*-8<1400:
            sleep(0.01)
            print(y*-8,y,sep=":::")
            set_rc_channel_pwm(3,y*-8)
    except KeyboardInterrupt:
        disarm()
        break