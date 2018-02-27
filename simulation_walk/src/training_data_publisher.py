#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from simulation_walk.msg import Laser4


class TrainingDataCollector:

    def __init__(self):
        self._training_laser_scan_msg = Laser4()
        self._depth = 4
        self._length = 662
        self._training_laser_scan_msg.depth = 4
        self._training_laser_scan_msg.length = 662
        self._publisher = rospy.Publisher(
            "simulation_walk/laser4", Laser4, queue_size=10)

    def laser_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header)
        self._training_laser_scan_msg.ranges.extend(list(data.ranges))
        # print len(self._training_laser_scan_msg.ranges)
        if (len(self._training_laser_scan_msg.ranges) ==
                self._depth * self._length):
            self._publisher.publish(self._training_laser_scan_msg)
            self._training_laser_scan_msg.ranges[:] = []

    def listner(self):
        rospy.init_node('listner', anonymous=True)

        rospy.Subscriber("/base_scan", LaserScan, self.laser_callback)
        rospy.spin()


if __name__ == '__main__':
    collector = TrainingDataCollector()
    collector.listner()
