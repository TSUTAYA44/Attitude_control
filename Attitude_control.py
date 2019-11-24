import smbus
import math
from time
import signal
import csv
from quaternion import kuaternion

DEV_ADDR = 0x68         # device address
ACCEL_XOUT = 0x3b
ACCEL_YOUT = 0x3d
ACCEL_ZOUT = 0x3f
TEMP_OUT = 0x41
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47
PWR_MGMT_1 = 0x6b       # PWR_MGMT_1
PWR_MGMT_2 = 0x6c       # PWR_MGMT_2

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
    x = ((x / 16384.0) *1)
    y = ((y / 16384.0) *1)
    z = ((z / 16384.0) *1)
    return [x, y, z]

class MadgwickAHRS:
    samplePeriod = 1/256
    quaternion = kuaternion(1, 0, 0, 0)
    beta = 1

    def __init__(self, sampleperiod=None, quaternion=None, beta=None):
        if sampleperiod is not None:
            self.samplePeriod = sampleperiod
        if quaternion is not None:
            self.quaternion = quaternion
        if beta is not None:
            self.beta = beta


    def update_imu(self, gyroscope, accelerometer):
        q = self.quaternion

        gyroscope = np.array(gyroscope, dtype=float).flatten()
        accelerometer = np.array(accelerometer, dtype=float).flatten()

        # Normalise accelerometer measurement
        if norm(accelerometer) is 0:
            warnings.warn("accelerometer is zero")
            return
        accelerometer /= norm(accelerometer)

        # Gradient descent algorithm corrective step
        f = np.array([
            2*(q[1]*q[3] - q[0]*q[2])   - accelerometer[0],
            2*(q[0]*q[1] + q[2]*q[3])   - accelerometer[1],
            2*(0.5 - q[1]**2 - q[2]**2) - accelerometer[2]
        ])
        j = np.array([
            [-2*q[2], 2*q[3]  ,-2*q[0], 2*q[1]],
            [2*q[1] , 2*q[0]  , 2*q[3], 2*q[2]],
            [0      ,-4*q[1]  ,-4*q[2],      0]
        ])
        step = j.T.dot(f)
        step /= norm(step)  # normalise step magnitude

        # Compute rate of change of quaternion
        qdot = (q * kuaternion(0, gyroscope[0], gyroscope[1], gyroscope[2])) * 0.5 - self.beta * step.T

        # Integrate to yield quaternion
        q += qdot * self.samplePeriod
        self.quaternion = kuaternion(q / norm(q))  # normalise quaternion

        quaternion = self.quaternion
        r, p, y = kuaternion.to_euler123(quaternion)

        return(r, p, y)

time = 0
dt   = 0
calculate_time = 10
gyroscope     = [0, 0, 0]
accelerometer = [0, 0, 0]

while 1:
    start = time.time()

    x_gyro,  y_gyro,  z_gyro  = get_gyro_data_deg()
    x_accel, y_accel, z_accel = get_accel_data_g()

    with open('measurement.csv', 'a') as measurement_file:
        writer = csv.DictWriter(measurement_file, fieldnames=fieldnames)
        writer.writerow({'x_accel':x_accel,
                         'y_accel':y_accel,
                         'z_accel':z_accel,
                         'x_gyro' :x_gyro ,
                         'y_gyro' :y_gyro ,
                         'z_gyro' :z_gyro ,
                         'dt'     :dt ,})

    dt = time.time() - start

    total_time += dt
    if total_time > calculate_time:
        break