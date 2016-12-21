#!/usr/bin/env python

import time
import serial
import numpy as np

import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import Imu
#from geometry_msgs.msg import Quaternion

def get_param(name, default):
	try:
		v = rospy.get_param(name)
		rospy.loginfo("Found parameter: %s, value: %s"%(name, str(v)))
	except KeyError:
		v = default
		rospy.logwarn("Cannot find value for parameter: %s, assigning "
				"default: %s"%(name, str(v)))
	return v


class Stim300Driver(object):
    
    #secs = 0
    #nsecs = 0

    _baudrate = 921600
    _datagram_lengths = {
        chr(0x93): 40 - 2,  # Rate, acceleration, inclination
    }
    # Rate, acceleration, incliination

    topic_name = 'imu/data'
    frame_id = 'imu_link'
    device_id = '/dev/ttyUSB0'


    def __init__(self):
        rospy.Time.from_sec(time.time())
        self.device_id = get_param('~device_id', '/dev/ttyUSB0')
        self.topic_name = get_param('~topic_name', 'imu/data')
        self.frame_id = get_param('~frame_id', 'imu_link')

        

        self.serial = serial.Serial(self.device_id, baudrate=self._baudrate)

        rospy.loginfo("Found parameter: %s, value: %s"%(self.device_id, str(self.serial)))
        
        self.datagram_identifier = chr(0x93)
        self.last_msg = None
        self.skipped_msgs = 0

        #rospy.Time.from_sec(time.time())
        #now = rospy.get_rostime()
        #secs = now.secs
        #nsecs = now.nsecs

        self.imu_pub = rospy.Publisher(self.topic_name, Imu, queue_size=10) #IMU message


    def read(self):
        return self.serial.read(64 * 12)

    def sync(self):
        char = None
        k = 0
        while char != self.datagram_identifier:
            k += 1
            char = self.serial.read(1)
        return k

    def read_datagram(self):
        msg = self.serial.read(
            self._datagram_lengths[self.datagram_identifier]
        )

        start = 0
        # gyr
        gyro = np.fromstring(
            b'\x00' + msg[start + 0:start + 3][::-1] +
            b'\x00' + msg[start + 3:start + 6][::-1] +
            b'\x00' + msg[start + 6:start + 9][::-1],
            dtype='<i'
        ).astype(np.float) / (2 ** 14)

        self.last_msg = gyro

        start += 10

        # acc
        linear_acceleration = np.fromstring(
            b'\x00' + msg[start + 0:start + 3][::-1] +
            b'\x00' + msg[start + 3:start + 6][::-1] +
            b'\x00' + msg[start + 6:start + 9][::-1],
            dtype='<i'
        ).astype(np.float32) / (2 ** 19)
        start += 10

        # inc
        inclination = np.fromstring(
            b'\x00' + msg[start + 0:start + 3][::-1] +
            b'\x00' + msg[start + 3:start + 6][::-1] +
            b'\x00' + msg[start + 6:start + 9][::-1],
            dtype='<i'
        ).astype(np.float32) / (2 ** 22)

        #print '[{:13.6f},{:13.6f},{:13.6f}  {:13.6f},{:13.6f},{:13.6f}  {:13.6f},{:13.6f},{:13.6f}]'.format(
        #    gyro[0],gyro[1],gyro[2],
        #    linear_acceleration[0],linear_acceleration[1],linear_acceleration[2],
        #    inclination[0],inclination[1],inclination[2])
        h = Header()
        
        now = rospy.Time.now()

        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.frame_id
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = gyro[1]
        imu_msg.angular_velocity.z = gyro[2]
        imu_msg.linear_acceleration.x = linear_acceleration[0]
        imu_msg.linear_acceleration.y = linear_acceleration[1]
        imu_msg.linear_acceleration.z = linear_acceleration[2]
        #all time assignments (overwriting ROS time)
        # Comment the two lines below if you need ROS time

        self.imu_pub.publish(imu_msg)
        #rospy.loginfo("Current time %i %i", imu_msg.header.stamp.secs, imu_msg.header.stamp.nsecs)
        #rospy.loginfo('{}--{}'.format(str(now),str(imu_msg)))

    def run(self):
        l = self.sync()
        if l != 38:
            return
        self.read_datagram()


def main():
    rospy.init_node('stim300_node')

    i = Stim300Driver()
    while(True):
        i.run()

if __name__ == '__main__':
    main()
