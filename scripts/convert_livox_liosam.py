#!/usr/bin/env python3

import rospy
import numpy as np
import json
from sensor_msgs.point_cloud2 import create_cloud
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import sys

class LiDARSubscriber:
    def __init__(self):

        self.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
            PointField('ring', 16, PointField.UINT16, 1)
        ]

        lidar_topic = '/livox/lidar'
        args = rospy.myargv(argv=sys.argv)
        if len(args)!=2:
            print("Missing lidar_topic argument, using default")
        elif len(args) == 2:
            lidar_topic = args[1]

        rospy.init_node('livox_msg_liosam', anonymous=True)
        self.lidar_pub = rospy.Publisher("/livox_points", PointCloud2, queue_size=1)
        rospy.Subscriber(lidar_topic, PointCloud2, self.lidar_callback)


    def lidar_callback(self, data):
        # This function will be called every time a new message is received on the lidar_topic
        # Access LiDAR data from the PointCloud2 message
        header = data.header
        # header.frame_id = 'velodyne'
        lidar_data = np.array([[x, y, z, intensity, line] for x, y, z, intensity, line in point_cloud2.read_points(data, field_names=("x", "y", "z", "intensity", "line"), skip_nans=True)])
        ring = lidar_data[:,3].astype(np.uint16)
        ring = ring.reshape(-1, 1)
        lidar_data = np.delete(lidar_data, obj=3, axis=1)        
        lidar_data = lidar_data.tolist()
        [lidar_data[i].insert(4, *ring[i]) for i in range(len(lidar_data))]
        point_cloud_msg = create_cloud(header, self.fields, lidar_data)
        self.lidar_pub.publish(point_cloud_msg)
        

    def run(self):
        rospy.spin()  # Keeps the node running

if __name__ == '__main__':
    try:
        lidar_subscriber = LiDARSubscriber()
        lidar_subscriber.run()
    except rospy.ROSInterruptException:
        pass
