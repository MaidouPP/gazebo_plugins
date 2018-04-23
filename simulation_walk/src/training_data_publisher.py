#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from simulation_walk.msg import Laser4, CostMap
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

        self._training_grid_msg = CostMap()
        self._len_map = 100
        self._resol = 0.05

        self._publisher = rospy.Publisher(
            "simulation_walk/costmap", CostMap, queue_size=10)
        self._sub_new_start = rospy.Subscriber(
            "/new_start", Point, self._gazebo_callback_new_start)
        self._sub_end_traj = rospy.Subscriber(
            "/reach_dest", Bool, self._gazebo_callback_end_traj)
        self._sub_end_traj = rospy.Subscriber(
            "/bump", Bool, self._gazebo_callback_bump)

    def _prepare_costmap(self, data):
        costmap = np.zeros((self._len_map**2))
        for i in xrange(len(list(data.ranges)):
            x = data.ranges[i] * math.cos(data.angle_min
                                        + i * data.angle_increment)
            y = data.ranges[i] * math.sin(data.angle_min
                                        + i * data.angle_increment)

            x = int(round(float(x)/self._resol + self._len_map/2))
            y = int(round(float(y)/self._resol + self._len_map/2))

            if x >= 0 and x < self._len_map and y >= 0 and y < self._len_map:
                costmap[x + self._len_map * y] = 1
        return costmap

    def laser_callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header)
        if self._ready:
            self._training_grid_msg.dara.extend(list(costmap))
            # print len(self._training_laser_scan_msg.ranges)
            if (len(self._training_grid_msg.data) ==
                    self._len_map**2):
                self._publisher.publish(self._training_grid_msg)
                self._training_grid_msg.data[:] = []

    def listner(self):
        rospy.init_node('listener', anonymous=True)

        rospy.Subscriber("/base_scan", LaserScan, self.laser_callback)
        rospy.spin()

    def _gazebo_callback_end_traj(self, data):
        self._ready = False
        self._training_laser_scan_msg = Laser4()

    def _gazebo_callback_new_start(self, data):
        self._ready = True

    def _gazebo_callback_bump(self, data):
        self._ready = False
        self._training_laser_scan_msg = Laser4()


if __name__ == '__main__':
    collector = TrainingDataCollector()
    collector.listner()
