# Import mavutil
from pymavlink import mavutil

# connection olusturma
master = mavutil.mavlink_connection(
            '/dev/ttyACM0',
            baud=115200)# Raspberry pi ile pixhawk'ın iletişim kurabilmesi için

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

# RC pwm değerlerini olusturuyoruz
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

deger= int(input("Deger Giriniz: ")) #pwm değeri 
#1100 Maximum ileri geri
#1900 Maximum hızda ileri
#1500 = 0

pin= int(input("Channel giriniz: ")) #komutlari integer olarak giriniz, komutları buradan ogrenebilirsiniz https://www.ardusub.com/operators-manual/rc-input-and-output.html
count = 0
while (count < 10000):
  set_rc_channel_pwm(pin, deger)
  count = count + 1