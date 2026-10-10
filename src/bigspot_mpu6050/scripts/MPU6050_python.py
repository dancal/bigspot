#! /usr/bin/env python3

from MPU6050 import MPU6050
import time
import rospy
import tf
import numpy as np
from sensor_msgs.msg import Imu
from MPUConstants import MPUConstants as C
from imu_units import raw_to_si, remap_mount
#from tf.transformations import *

class Kalman():

    def __init__(self):
        self.P = np.matrix([[0., 0.],[0., 0.]])
    def setKalmanAngle(self, angle):
        self.State = np.matrix([[angle],[0.   ]])
        #print('0. set initial guess: \n', self.State)

    def getKalmanAngle(self, angle, gyro_rate, dt):
        R = 0.03
        Q = np.matrix([[0.001, 0.   ],[0.,    0.003]])
        H = np.matrix( [1.,    0.   ])

        F = np.matrix([[1., -dt],[0., 1. ]])
        B = np.matrix([[dt],[0.]])
        #print('F= \n', F)
        #print('B= \n', B)
        #print(self.State)
        
        #(I). State prediction
        self.State = F * self.State + B * gyro_rate
        #print('I. State prediction: \n', self.State)

        #(II). Covariance prediction
        self.P = F * self.P * np.transpose(F) + Q
        #print('II. Covariance prediction P: \n', self.P)

        #(III). Innovation
        I = angle - H * self.State
        #print('III. Innovation I: \n', I)

        #(IV). Innovation covariance S
        S = H * self.P * np.transpose(H) + R
        #print('IV. Innovation covariance S: \n', S)

        #(V). Kalman gain KG
        KG = self.P * np.transpose(H) / S
        #print('V. Kalman Gain: \n', KG)

        #(VI). Update state
        self.State = self.State + KG * I
        #print('VI. Update State: \n', self.State)

        #(VII). Update covariance
        self.P = (np.eye(2) - KG * H) * self.P
        #print('VII. Update Covariance P: \n', self.P)

        return self.State.item(0)

#X = Kalman()
#X.setKalmanAngle(30)
#x1 = X.getKalmanAngle(30, 2, 0.01)
#print(x1)

if __name__ == "__main__":
    rospy.init_node("bigspot_mpu6050")
    pub         = rospy.Publisher("bigspot_imu/base_link_orientation", Imu,queue_size=10)
    mount_rpy=np.deg2rad(rospy.get_param('~sensor_mount_rpy_deg',[0.,0.,0.]))
    mount_quaternion=tf.transformations.quaternion_from_euler(*mount_rpy)

    i2c_bus = 1
    device_address = 0x68

    # Biases belong to the individual IMU. Do not reuse the original author's calibration.
    x_accel_offset,y_accel_offset,z_accel_offset=map(int,rospy.get_param('~accel_offsets',[0,0,0]))
    x_gyro_offset,y_gyro_offset,z_gyro_offset=map(int,rospy.get_param('~gyro_offsets',[0,0,0]))

    #x_avg_read: 684.76 x_avg_offset: -1628.91225
    #y_avg_read: 1436.4 y_avg_offset: -3390.4574999999995
    #z_avg_read: 211.12 z_avg_offset: -519.4865000000003

    #x_avg_read: 58.51 x_avg_offset: -131.9548125
    #y_avg_read: 13.16 y_avg_offset: -29.24875
    #z_avg_read: -9.31 z_avg_offset: 20.897812500000004

    #x_avg_read: -0.26 x_avg_offset: -1711.2001250000003
    #y_avg_read: -0.3 y_avg_offset: -3590.031374999994
    #z_avg_read: -0.2 z_avg_offset: -527.3312500000011

    #x_avg_read: 0.21 x_avg_offset: -146.85118750000032
    #y_avg_read: -0.08 y_avg_offset: -32.400249999999936
    #z_avg_read: 0.23 z_avg_offset: 23.181937500000004


    enable_debug_output = False
    mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,enable_debug_output)

    mpu.dmp_initialize()
    gyro_fs=mpu.read_bits(C.MPU6050_RA_GYRO_CONFIG,C.MPU6050_GCONFIG_FS_SEL_BIT,C.MPU6050_GCONFIG_FS_SEL_LENGTH)
    accel_fs=mpu.read_bits(C.MPU6050_RA_ACCEL_CONFIG,C.MPU6050_ACONFIG_AFS_SEL_BIT,C.MPU6050_ACONFIG_AFS_SEL_LENGTH)
    mpu.set_DMP_enabled(True)
    mpu_int_status = mpu.get_int_status()
    print(hex(mpu_int_status))

    packet_size = mpu.DMP_get_FIFO_packet_size()
    print(packet_size)
    FIFO_count = mpu.get_FIFO_count()
    print(FIFO_count)

    #mpu.calibrateMPU6500()
    # print(self.gbias)
    # print(self.abias)
    #Roll = Kalman()
    #Pitch = Kalman()
    #Yaw = Kalman()
    #Roll.setKalmanAngle(0)
    #Pitch.setKalmanAngle(0)
    #Yaw.setKalmanAngle(0.)

    FIFO_buffer = [0]*64
    rate = rospy.Rate(100)
    warmup_until=time.monotonic()+2.
    #make initial guesses
    #time_pre = time.time()

    bActive     = False
    mpu.reset_FIFO()
    #print('AVG = ', roll_pitch_yaw_x, roll_pitch_yaw_y, roll_pitch_yaw_z)
    try:
        Count   = 0
        while not rospy.is_shutdown():
            Count += 1

            try:
                FIFO_count = mpu.get_FIFO_count()
                mpu_int_status = mpu.get_int_status()
            except OSError as error:
                rospy.logwarn_throttle(2.,'IMU read failed: '+str(error))
                rate.sleep()
                continue

            # If overflow is detected by status or fifo count we want to reset
            if (FIFO_count == 1024) or (mpu_int_status & 0x10):
                mpu.reset_FIFO()
                #print('overflow!')
            # Check if fifo data is ready
            elif (mpu_int_status & 0x02):
                # Wait until packet_size number of bytes are ready for reading, default
                # is 42 bytes
                deadline=time.monotonic()+.05
                while FIFO_count < packet_size and time.monotonic()<deadline and not rospy.is_shutdown():
                    FIFO_count = mpu.get_FIFO_count()
                    time.sleep(.001)
                if FIFO_count < packet_size:
                    rate.sleep()
                    continue

                # Discard old complete packets; control must receive the newest orientation.
                while FIFO_count >= packet_size:
                    FIFO_buffer=mpu.get_FIFO_bytes(packet_size)
                    FIFO_count-=packet_size
                #accel           = mpu.DMP_get_acceleration_int16(FIFO_buffer)
                quat            = mpu.DMP_get_quaternion(FIFO_buffer)
                grav            = mpu.DMP_get_gravity(quat)
                roll_pitch_yaw  = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
                #acc             = mpu.get_acceleration()
                #gyro            = mpu.get_rotation()

                # DMP_get_quaternion
                #  attitude = ahrs.filters.Madgwick(acc=acc_data, gyr=gyro_data)
                #accVal          = [acc[0]/16384.0, acc[1]/16384.0, acc[2]/16384.0]
                #gryoVal         = [gyro[0]/16.384, gyro[1]/16.384, gyro[2]/16.384]
                
                #roll_pitch_yaw.z    -= 5.01

                if time.monotonic()<warmup_until:
                    rate.sleep()
                    continue

                # covariance matrix
                imu_data                        = Imu()
                imu_data.header.stamp           = rospy.Time.now()

                imu_data.header.frame_id = 'base_link'
                values = np.array([quat.x, quat.y, quat.z, quat.w], dtype=float)
                norm = np.linalg.norm(values)
                if not np.isfinite(values).all() or norm < .5:
                    continue
                values /= norm
                imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w = values
                imu_data.orientation_covariance = [0.]*9
                try:
                    angular,acceleration=raw_to_si(mpu.get_rotation(),mpu.get_acceleration(),gyro_fs,accel_fs)
                    values,angular,acceleration=remap_mount(values,angular,acceleration,mount_quaternion)
                except (OSError,ValueError) as error:
                    rospy.logwarn_throttle(2.,'IMU sample rejected: '+str(error))
                    rate.sleep()
                    continue
                imu_data.angular_velocity.x,imu_data.angular_velocity.y,imu_data.angular_velocity.z=angular
                imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.z,imu_data.orientation.w=values
                imu_data.linear_acceleration.x,imu_data.linear_acceleration.y,imu_data.linear_acceleration.z=acceleration
                # All-zero covariance means unknown, not perfect measurement.
                # Sensor mounting must align x forward, y left, z up before motor use.
                #imu_data.orientation.z          = quaternion[2]
                #imu_data.orientation.w          = quaternion[3]

                pub.publish(imu_data)

                #time_pre = time.time()
                rate.sleep()
                #rospy.loginfo(roll_pitch_yaw.x)
            else:
                rate.sleep()

    except KeyboardInterrupt:
        print('\n Ctrl + C QUIT')
