#!/usr/bin/env python3

import numpy as np
import rospy
import sys
import struct

from livox_msg_conversions.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

class message_converter:
    def __init__(self):


        args = rospy.myargv(argv=sys.argv)
        if len(args)!=2:
            print("Missing arguments vehicle_frame and json_config")
        elif len(args) == 2:
            lidar_topic = args[1]

        rospy.init_node('livox_msg_conversions', anonymous=True)
        self.lidar_pub = rospy.Publisher("/livox_points", CustomMsg, queue_size=1)
        rospy.Subscriber(lidar_topic, PointCloud2, self.lidar_callback)

    def lidar_callback(self, data):

        custom_msg_out = CustomMsg()
        
        pkg_base_time = data.header.stamp.to_sec() * 1000000000.0
        # packed_data = struct.pack('!d', pkg_base_time)

        custom_msg_out.header = data.header
        custom_msg_out.timebase = np.uint64(pkg_base_time).item()#struct.unpack('!Q', packed_data)[0]#np.uint64(pkg_base_time) 
        custom_msg_out.point_num = data.width
        custom_msg_out.lidar_id = 0
        points_all = []

        for x, y, z, intensity, tag, line, timestamp in point_cloud2.read_points(data, field_names=("x", "y", "z", "intensity", "tag", "line", "timestamp"), skip_nans=True):
            # print("data.header.stamp", custom_msg_out.timebase)
            # print("timestamp", timestamp - custom_msg_out.timebase)
            # print("timestamp", timestamp)
            offset_time_ = timestamp - pkg_base_time
            # packed_data = struct.pack('!f', timestamp - pkg_base_time)
            point = CustomPoint()
            point.x = x
            point.y = y
            point.z = z
            point.reflectivity = np.uint8(intensity).item()
            point.tag = tag
            point.line = line
            point.offset_time = np.uint32(offset_time_).item()#struct.unpack('!I', packed_data)[0]
            points_all.append(point)
        
        custom_msg_out.points = points_all
        self.lidar_pub.publish(custom_msg_out)

    def run(self):
        rospy.spin()  # Keeps the node running                                                           

if __name__ == '__main__':
    try:
        lidar_subscriber = message_converter()
        lidar_subscriber.run()
    except rospy.ROSInterruptException:
        pass