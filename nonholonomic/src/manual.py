#!/usr/bin/env python
import rospy
import sys
import time
import math
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
import tf.transformations

class Manual(object):
    def __init__(self, map, model, global_planner):
        self.map = map
        self.robot_model = model
        self.start = None
        self.goal = None
        self.global_planner = global_planner
        self.t0 = None
        self.t1 = None


    def setstart(self):
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.getPoseData)

    def getPoseData(self, data):
        startpose = data.pose.pose.position
        startquaternion = data.pose.pose.orientation
        #print("Point Position: [ %f, %f, %f ]"%(startpose.x, startpose.y, startpose.z))
        #print("Quat Orientation: [ %f, %f, %f, %f]"%(startquaternion.x, startquaternion.y, startquaternion.z, startquaternion.w))
        startquaternion_t = tf.transformations.euler_from_quaternion([startquaternion.x, startquaternion.y, startquaternion.z, startquaternion.w])
        #print("Euler Angles: %s"%str(startquaternion_t[2]))
        startangle = 0.0
        if startquaternion_t[2] > 0:
            startangle = 2* math.pi - startquaternion_t[2]
        elif startquaternion_t[2] < 0:
            startangle = - startquaternion_t[2]

        if 0 < startpose.x < self.map.max_x and 0 < startpose.y < self.map.max_y and self.start_Valid(startpose.x, self.map.max_y - startpose.y):
            self.start = (startpose.x, self.map.max_y - startpose.y, startangle)
            print("get start")
            print(self.start)

            #Publish the start pose --arrow
            start_pub = rospy.Publisher("/move_base_simple/start", PoseStamped, latch=True, queue_size=10)
            p = PoseStamped()
            p.header.stamp = rospy.Time.now()
            p.header.frame_id = "/map"
            p.pose.position.x = startpose.x
            p.pose.position.y = startpose.y
            p.pose.position.z = startpose.z
            (p.pose.orientation.x,
             p.pose.orientation.y,
             p.pose.orientation.z,
             p.pose.orientation.w) = (startquaternion.x, startquaternion.y, startquaternion.z, startquaternion.w)
            #tf.transformations.quaternion_from_euler(0, 0, self.start[2])
            #p.pose.covariance[6 * 0 + 0] = 0.5 * 0.5
            #p.pose.covariance[6 * 1 + 1] = 0.5 * 0.5
            #p.pose.covariance[6 * 3 + 3] = math.pi / 12.0 * math.pi / 12.0
            start_pub.publish(p)
            if self.start is not None and self.goal is not None:
                self.plan(self.start, self.goal)



    def setgoal(self):
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.getGoalData)

    def getGoalData(self,data):
        goalpose = data.pose.position
        goalquaternion = data.pose.orientation
        #print("Point Position: [ %f, %f, %f ]" % (goalpose.x, goalpose.y, goalpose.z))
        #print("Quat Orientation: [ %f, %f, %f, %f]" % (goalquaternion.x, goalquaternion.y, goalquaternion.z, goalquaternion.w))
        goalquaternion_t = tf.transformations.euler_from_quaternion([goalquaternion.x, goalquaternion.y, goalquaternion.z, goalquaternion.w])
        #print("Euler Angles: %s" % str(goalquaternion_t[2]))
        goalangle = 0.0
        if goalquaternion_t[2] > 0:
            goalangle = 2* math.pi - goalquaternion_t[2]
        elif goalquaternion_t[2] < 0:
            goalangle = - goalquaternion_t[2]

        if 0 < goalpose.x < self.map.max_x and 0 < goalpose.y < self.map.max_y and self.goal_Valid(goalpose.x, self.map.max_y - goalpose.y):
            self.goal = (goalpose.x, self.map.max_y - goalpose.y, goalangle)
            print("get goal")
            print(self.goal)
            if self.start is not None and self.goal is not None:
                self.plan(self.start, self.goal)


    def start_Valid(self, x, y):
        radius_in_grid = self.robot_model.radius / self.map.resolution
        if x < radius_in_grid or y < radius_in_grid or x +radius_in_grid >= self.map.max_x or y + radius_in_grid >= self.map.max_y:
            print('invalid start!!!')
            return False
        xmin = int(x - radius_in_grid)
        xmax = int(x + radius_in_grid)
        ymin = int(y - radius_in_grid)
        ymax = int(y + radius_in_grid)

        for xi in range(int(xmin), int(xmax)+1):
            for yi in range(int(ymin), int(ymax)+1):
                if self.map.grid_type(xi, yi) == 'occupied':
                    print('invalid start!!!')
                    return False

        return True


    def goal_Valid(self, x, y):
        radius_in_grid = self.robot_model.radius / self.map.resolution
        if x < radius_in_grid or y < radius_in_grid or x +radius_in_grid >= self.map.max_x or y + radius_in_grid >= self.map.max_y:
            print('invalid goal!!!')
            return False
        xmin = int(x - radius_in_grid)
        xmax = int(x + radius_in_grid)
        ymin = int(y - radius_in_grid)
        ymax = int(y + radius_in_grid)

        for xi in range(int(xmin), int(xmax)+1):
            for yi in range(int(ymin), int(ymax)+1):
                if self.map.grid_type(xi, yi) == 'occupied':
                    print('invalid goal!!!')
                    return False

        return True

    def plan(self, start, goal):
        self.t0 = rospy.get_time()
        #print(self.t0)
        path = self.global_planner.get_path(start, goal, self.map)
        self.global_planner.show_path('rviz_global_path', self.map)
        #self.global_planner.send_path('global_path', self.map)
        self.global_planner.show_nodes('rviz_nodes', self.map)
        self.t1 = rospy.get_time()
        #print(self.t1)
        print('total time:', self.t1 - self.t0)
        print('----------------------')
        #print(path)


