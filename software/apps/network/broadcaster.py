# RUN THIS AS ROOT!
# python3 ./broadcaster.py
import subprocess
import time


def start_broadcasting():
    print("Start")
    subprocess.run("hcitool -i hci0 cmd 0x08 0x000a 01", shell=True)
    time.sleep(0.1)

def stop_broadcasting():
    print("Stop")
    subprocess.run("hciconfig hci0 noleadv", shell=True)
    time.sleep(0.5)

def set_broadcast_rate():
    print("Set Rate")
    # subprocess.run("hcitool -i hci0 cmd 0x08 0x0006 A0 00 A0 00 03 00 00 00 00 00 00 00 00 07 00", shell=True)
    subprocess.run("hcitool -i hci0 cmd 0x08 0x0006 00 01 00 01 03 00 00 00 00 00 00 00 00 07 00", shell=True)
    time.sleep(0.1)

def set_mac_address():
    # OLD MAC: 5C:F3:70:9C:52:62
    print("Change MAC")
    subprocess.run("hcitool cmd 0x3f 0x001 0x54 0x53 0x9c 0x70 0xf3 0xc0", shell=True)
    time.sleep(0.1)

def set_broadcast_advdata(data):
    print("Set Data")
    # base_cmd = ['hcitool', '-i', 'hci0', 'cmd', '0x08', '0x0008', '11', '02', '01', '1a', '0d', 'ff']
    # set_broadcast_advdata(['E0', '02', '48', '45', '4c', '4c', '4f', '57', '4f', '52', '4c', '44'])
    base_cmd = ['hcitool', '-i', 'hci0', 'cmd', '0x08', '0x0008', '1f', '02', '01', '06', '1b', 'ff', 'e0', '02']
    base_cmd.extend(data)
    subprocess.run(base_cmd)
    time.sleep(0.1)

#* payload[2] = LW_speed,
#* payload[3] = LW_dir
#* payload[4] = RW_speed,
#* payload[5] = RW_dir
#* payload[6] = Switch (Not used)
#* payload[7] = Lift_PWM (Not used)
#* payload[8] = Tilt_PWM (Not used)
#* payload[9] = Gripper_bool
#* payload[10] = Lift_speed_tick
#* payload[11] = Lift_speed_dir
#* payload[12] = Tilt_speed_tick
#* payload[13] = Tilt_speed_dir

def main_test_changing_ws():
    lw_speed = 0
    payload = ['ff'] * 26
    payload[0] = 'e0'
    payload[1] = '02'
    payload[2] = '00'
    payload[3] = '00'
    stop_broadcasting()
    set_mac_address()
    set_broadcast_rate()
    start_broadcasting()
    for _ in range(50):
        print("===DATA-2 is %s" % payload[2])
        set_broadcast_advdata(payload[2:])
        time.sleep(0.1)
        lw_speed += 1
        payload[2] = "{:02x}".format(lw_speed)

    stop_broadcasting()
    print("END")


def main_test_arm_lift():
    data = ['ff'] * 26
    data[0] = 'e0'
    data[1] = '02'
    data[10] = '32'
    data[11] = '01'
    stop_broadcasting()
    set_mac_address()
    set_broadcast_rate()
    start_broadcasting()
    for _ in range(10):
        print("===DATA-11 is %s" % data[11])
        set_broadcast_advdata(data[2:])
        time.sleep(3)
        if data[11] == '01':
            data[11] = '00'
        else:
            data[11] = '01'
    
    stop_broadcasting()
    print("END")

if __name__ == '__main__':
    main_test_changing_ws()
    # main_test_arm_lift()





