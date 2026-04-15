
#!/usr/bin/env python
# license removed for brevity
import rospy
from sensor_msgs.msg import *

import serial

ser = serial.Serial(
    port='/dev/matrix_icm20948_imu',
    baudrate=115200
)

def getGyroData(raw_data):
    gyro = []
    i = 0
    keyword = ' g[ '
    for x in range(3):
        try:
            gyro_raw = raw_data[raw_data.find(keyword)+ len(keyword)+i:raw_data.find(keyword) + len(keyword) + 6+i]
            gyro_raw = float(gyro_raw)
        except:
            gyro_raw = 0.0
        gyro.append(gyro_raw)
        i+=8
    if abs(gyro[-1]) <= 0.077:
        gyro[-1] = 0.0
    return gyro

def getAccelData(raw_data):
    acc = []
    i = 0
    keyword = 'a[ '
    for x in range(3):
        try:
            acc_raw = raw_data[raw_data.find(keyword)+ len(keyword)+i:raw_data.find(keyword) + len(keyword) + 8+i]
            acc_raw = float(acc_raw)
        except:
            acc_raw = 0.0
        acc.append(acc_raw)
        i+=10
    return acc

def getMagData(raw_data):
    mag = []
    i = 0
    keyword = 'm[ '
    for x in range(3):
        try:
            mag_raw = raw_data[raw_data.find(keyword)+ len(keyword)+i:raw_data.find(keyword) + len(keyword) + 10+i]
            mag_raw = float(mag_raw)
        except:
            mag_raw = 0.0
        mag.append(mag_raw)
        i+=12
    return mag

def getTempData(raw_data):
    temp = []
    i = 0
    keyword = 't[ '
    for x in range(1):
        try:
            temp_raw = raw_data[raw_data.find(keyword)+ len(keyword)+i:raw_data.find(keyword) + len(keyword) + 9]
            temp_raw = float(temp_raw)
        except:
            temp_raw = 0.0
        temp.append(temp_raw)
    return temp


# while(True):
#     data = str(ser.readline())
#     print("Raw data ICM20948: ", data)
#     print("Accel(m/s^2): ", getAccelData(data))
#     print("Gyro(rad/s): ", getGyroData(data))
#     print("Mag(T): ", getMagData(data))
#     print("Temp(c): ", getTempData(data))
#     print("*****")

# ser.close()

def talker():
    rospy.init_node('imu_icm20948', anonymous=True)
    pub = rospy.Publisher('imu_icm20948/data_raw', Imu, queue_size=1)
    pub_temp = rospy.Publisher('imu_icm20948/tempurature', Temperature, queue_size=1)
    pub_mag = rospy.Publisher('imu_icm20948/mag', MagneticField, queue_size=1)
    rate = rospy.Rate(100) # 10hz

    rospy.loginfo("[imu_icm20948]: Start publishing data...")
    while not rospy.is_shutdown():
        try:
            data = str(ser.readline())

            acc = getAccelData(data)
            gyro = getGyroData(data)
            mag = getMagData(data)
            temp = getTempData(data)


            imu_data = Imu()
            imu_data.header.stamp = rospy.Time.now()
            imu_data.header.frame_id = "imu_mount_point_link"
            imu_data.angular_velocity.x = gyro[0]
            imu_data.angular_velocity.y = gyro[1]
            imu_data.angular_velocity.z = gyro[2]
            imu_data.linear_acceleration.x = acc[0]
            imu_data.linear_acceleration.y = acc[1]
            imu_data.linear_acceleration.z = acc[2]
            # print(imu_data)
            pub.publish(imu_data)

            temp_data = Temperature()
            temp_data.temperature = temp[0]
            pub_temp.publish(temp_data)

            mag_data = MagneticField()
            mag_data.header.stamp = rospy.Time.now()
            mag_data.header.frame_id = "imu_mount_point_link"
            mag_data.magnetic_field.x = mag[0]
            mag_data.magnetic_field.y = mag[1]
            mag_data.magnetic_field.z = mag[2]
            pub_mag.publish(mag_data)

        except:
            print("mcu disconnect")
        
        rate.sleep()
    ser.close()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass