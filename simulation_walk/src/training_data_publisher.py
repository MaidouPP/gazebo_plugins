#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from simulation_walk.msg import Laser4
from std_msgs.msg import Bool
from geometry_msgs.msg import Point


class TrainingDataCollector:

    def __init__(self):
        self._training_laser_scan_msg = Laser4()
        self._depth = 4
        self._length = 662
        self._training_laser_scan_msg.depth = 4
        self._training_laser_scan_msg.length = 662
        self._ready = False
        self._publisher = rospy.Publisher(
            "simulation_walk/laser4", Laser4, queue_size=10)
        self._sub_new_start = rospy.Subscriber(
            "/new_start", Point, self._gazebo_callback_new_start)
        self._sub_end_traj = rospy.Subscriber(
            "/reach_dest", Bool, self._gazebo_callback_end_traj)

    def laser_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header)
        if self._ready:
            self._training_laser_scan_msg.ranges.extend(list(data.ranges))
            # print len(self._training_laser_scan_msg.ranges)
            if (len(self._training_laser_scan_msg.ranges) ==
                    self._depth * self._length):
                self._publisher.publish(self._training_laser_scan_msg)
                self._training_laser_scan_msg.ranges[:] = []

    def listner(self):
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("/base_scan", LaserScan, self.laser_callback)
        rospy.spin()

    def _gazebo_callback_end_traj(self, data):
        self._ready = False
        self._training_laser_scan_msg = Laser4()

    def _gazebo_callback_new_start(self, data):
        self._ready = True

if __name__ == '__main__':
    collector = TrainingDataCollector()
    collector.listner()
