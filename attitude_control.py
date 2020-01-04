import smbus
import math
import time
import csv
import pigpio
from pigpio import pi

DEV_ADDR = 0x68
ACCEL_XOUT = 0x3b
ACCEL_YOUT = 0x3d
ACCEL_ZOUT = 0x3f
TEMP_OUT = 0x41
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47
PWR_MGMT_1 = 0x6b
PWR_MGMT_2 = 0x6c

bus = smbus.SMBus(1)
bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0)

def read_byte(adr):
    return bus.read_byte_data(DEV_ADDR, adr)

def read_word(adr):
    high = bus.read_byte_data(DEV_ADDR, adr) 
    low  = bus.read_byte_data(DEV_ADDR, adr+1)
    val  = (high << 8) + low
    return val

def read_word_sensor(adr):
    val  = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def get_gyro_data_lsb():               #角速度(ジャイロ)データ取得
    x = read_word_sensor(GYRO_XOUT)
    y = read_word_sensor(GYRO_YOUT)
    z = read_word_sensor(GYRO_ZOUT)
    return [x, y, z]

def get_gyro_data_deg():
    x,y,z = get_gyro_data_lsb()
    x = x / 1310
    y = y / 1310
    z = z / 1310
    return [x, y, z]

def get_accel_data_lsb():              #加速度データ取得
    x = read_word_sensor(ACCEL_XOUT)
    y = read_word_sensor(ACCEL_YOUT)
    z = read_word_sensor(ACCEL_ZOUT)
    return [x, y, z]

def get_accel_data_g():
    x,y,z = get_accel_data_lsb()
    x = x / 16384.0
    y = y / 16384.0
    z = z / 16384.0
    return [x, y, z]

class Servo_Class:
    def __init__(self, Pin):
        self.pi = pi()
        self.mPin = Pin
        self.pi.set_mode(self.mPin, pigpio.OUTPUT)

    def SetPos(self, degree):
        duty = int(((12-2.5)/180*degree+2.5)*10000)
        self.pi.hardware_PWM(self.mPin, 50, duty)

    def Cleanup(self):
        self.pi.hardware_PWM(self.mPin, 50, self.SetPos(0))
        time.sleep(1)
        self.pi.set_mode(self.mPin, pigpio.INPUT)
        self.pi.stop()

dt = 0 #初期値の設定
total_time = 0
total_counts = 0
calculate_time= 50
old_y_gyro = 0
old_angle = 0
fieldnames = ['x_accel', 'y_accel', 'z_accel', 'x_gyro', 'y_gyro', 'z_gyro', 'dt']

Servo = Servo_Class(Pin=18) #Servo_Classのインスタンス化
Servo.SetPos(90) #初期位置を90度に設定

while 1:
    start = time.time() #ループ中にかかる時間を計測開始
    
    x_gyro,  y_gyro,  z_gyro  = get_gyro_data_deg()
    x_accel, y_accel, z_accel = get_accel_data_g()

    y_gyro = y_gyro*0.3
    y_angle = math.atan2(x_accel , math.sqrt(y_accel**2 + z_accel**2))#姿勢角の算出
    k = 0.2 * (65536**-((abs(x_accel))/10)) #加速度が増加→加速度から得られる角度の比重を小さくしていく係数k
    y_angle = (((1-k) * (old_angle + (((old_y_gyro + y_gyro)* dt) / 2)) + ((k* y_angle)))) #加速度とジャイロの相補フィルター
    old_angle = y_angle #角速度で足していくため一つ前の角度が必要
    old_y_gyro = y_gyro #台形うよう積分用に一つ前の角速度を残しておく

    if int(total_time) % 1 == 0: #dt秒毎に処理を実行
        if 86-int(math.degrees(y_angle)) < 180 and 86-int(math.degrees(y_angle)) > 0: #0度~180度の時のみ動作
            Servo.SetPos(86 - int(math.degrees(y_angle))) #初ｓ期位置は90度からなので(86は調整した)

    with open('measurement.csv', 'a') as measurement_file:
        writer = csv.DictWriter(measurement_file, fieldnames=fieldnames)
        writer.writerow({'x_accel':x_accel,
                         'y_accel':y_accel,
                         'z_accel':z_accel,
                         'x_gyro' :x_gyro ,
                         'y_gyro' :y_gyro ,
                         'z_gyro' :z_gyro ,
                         'dt'     :dt })

    if int(total_time) == int(calculate_time): #計測時間がtotal_timeに達したらプログラム終了
        break

    dt = time.time() - start #ループにかかる時間を微小時間dtに代入
    total_time += dt

Servo_right.Cleanup()
