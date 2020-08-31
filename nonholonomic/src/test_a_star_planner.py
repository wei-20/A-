#!/usr/bin/env python
import rospy
import yaml
import time
import math
import numpy as np
from a_star_planner import AStarPlanner
from robot_model import RobotModel
import matplotlib.image as mpimg
from grid_map import GridMap
from manual import Manual

rospy.init_node('test_a_star_planner', anonymous=True)

vehicle_config = yaml.load(open('/home/ww/catkin_ws/src/a_star_planner/vehicle_config.yaml'))
half_length = 0.5 * vehicle_config['length']
half_width = 0.5 * vehicle_config['width']
radius = math.sqrt(half_length ** 2 + half_width ** 2)
robot_model = RobotModel(radius)
global_planner = AStarPlanner(robot_model)

free = mpimg.imread('/home/ww/catkin_ws/src/a_star_planner/map_demo.png')
global_map_config = yaml.load(open('/home/ww/catkin_ws/src/a_star_planner/map_demo.yaml'))
global_map = GridMap(free, global_map_config['resolution'])
global_map.show('rviz_global_grid_map')
# global_map.print()



manual = Manual(global_map, robot_model, global_planner)
print('-----Please set the start and the goal-----')
manual.setstart()
manual.setgoal()

while not rospy.core.is_shutdown():
    time.sleep(1.0)
    

            #global_planner.show_path('rviz_global_path', global_map)
            #global_planner.send_path('global_path', global_map)
            #global_planner.show_nodes('rviz_nodes', global_map)
            # global_planner.show_vehicles('rviz_vehicles', global_map)

print('Finished!')


#global_mission = yaml.load(open('/home/ww/catkin_ws/src/a_star_planner/global_mission.yaml'))

#start_grid_x = (global_mission['start'][0] / global_map.resolution)
#start_grid_y = (global_mission['start'][1] / global_map.resolution)
#start_grid_t = (global_mission['start'][2]) /180 *math.pi
#goal_grid_x = global_mission['goal'][0] / global_map.resolution
#goal_grid_y = (global_mission['goal'][1] / global_map.resolution)
#goal_grid_t = (global_mission['goal'][2]) /180 *math.pi
#start = (start_grid_x, start_grid_y, start_grid_t)
#goal = (goal_grid_x, goal_grid_y, goal_grid_t)

#path = global_planner.get_path(start, goal, global_map)
#global_planner.show_path('rviz_global_path', global_map)
#global_planner.send_path('global_path', global_map)
#global_planner.show_nodes('rviz_nodes', global_map)
#print(path)
#print(start,goal)
#print('OK!')


#while not rospy.core.is_shutdown():
    #time.sleep(1.0)
    #global_planner.send_path('global_path', global_map)
    #global_planner.show_path('rviz_global_path', global_map)
    #global_planner.show_nodes('rviz_nodes', global_map)

#print('Finished!')
#print(goal)
#print(global_map.max_x,global_map.max_y)
#print(global_map.occupancy(9,8))
