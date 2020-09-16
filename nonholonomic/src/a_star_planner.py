import rospy
import math
import sys
import reeds_shepp
import matplotlib.pyplot as plt
from robot_model import RobotModel
from grids_info import GridsInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Path

class AStarPlanner(object):
    ''' Traditional A* planner.

    See: https://github.com/karlkurzer/path_planner

    Arguments
    ---------
        robot_model (RobotModel): The robot model used for planning.
    '''


    def __init__(self, robot_model):
        # Member
        self.path = None
        self.open_list = None
        self.close_list = None
        self.grids_info = None
        self.robot_model = robot_model
        self.iteration_max = 30000
        self.iterations = 0
        self.rho = 1.6 / math.tan(0.2292)



    def ready_for_search(self, start_grid, goal_grid, grid_map):
        # Initialize the path
        self.path = []

        # Initialize open list and close list.
        self.open_list = []
        grid = (int(start_grid[0]), int(start_grid[1]), int(start_grid[2] / 0.206))
        self.open_list.append(grid)
        self.close_list = []

        # Initialize all grids' information.
        self.grids_info = GridsInfo(grid_map.max_x, grid_map.max_y)
        self.grids_info.set_state(start_grid[0], start_grid[1], start_grid[2])
        id_grid = self.grids_info.get_idx(grid[0], grid[1], grid[2])
        self.grids_info.set_g_cost(id_grid, 0.0)
        self.grids_info.set_h_cost(id_grid, self.get_heuristic(start_grid, goal_grid))



    def collision(self, grid, grid_map):
        radius_in_grid = int(self.robot_model.radius / grid_map.resolution + 1)
        #radius_in_grid = self.robot_model.radius / grid_map.resolution
        for dx in range(-radius_in_grid, radius_in_grid+1):
            for dy in range(-radius_in_grid, radius_in_grid+1):
                ddis_in_grid = dx**2 + dy**2
                if ddis_in_grid > radius_in_grid**2:
                    continue

                # Here is grid within robot model.
                x = int(grid[0]) + dx
                y = int(grid[1]) + dy
                if grid_map.grid_type(x, y) == 'occupied':
                    return True

        return False



    def get_transition_cost(self, grid, succ_grid, grid_map, prim):
        #succ_grid = (grid[0]+u[0], grid[1]+u[1])
        if self.collision(succ_grid, grid_map):
            return float('inf')
        elif prim == 0:
            return math.sqrt((succ_grid[0] - grid[0]) ** 2 + (succ_grid[1] - grid[1]) ** 2)
        elif 0 < prim < 3:
            return math.sqrt((succ_grid[0] - grid[0]) ** 2 + (succ_grid[1] - grid[1]) ** 2) + 0.01
        elif prim == 3:
            return math.sqrt((succ_grid[0] - grid[0]) ** 2 + (succ_grid[1] - grid[1]) ** 2) * 1.2
        else:
            return math.sqrt((succ_grid[0] - grid[0]) ** 2 + (succ_grid[1] - grid[1]) ** 2) * 1.2 + 0.01
        #elif prim <5:
        #    return math.sqrt((succ_grid[0] - grid[0]) ** 2 + (succ_grid[1] - grid[1]) ** 2) + 0.02


    def get_heuristic(self, grid, goal_grid):
        #dx = goal_grid[0] - grid[0]
        #dy = goal_grid[1] - grid[1]
        #h_state = math.sqrt(dx ** 2 + dy ** 2)
        #dist = 0.0
        #if self.iterations % 1000 == 0:
        #    dist = reeds_shepp.path_length(grid, goal_grid, self.rho)
        dist = reeds_shepp.path_length(grid, goal_grid, self.rho)
        return dist



    def trace_back(self, grid):
        id = self.grids_info.get_idx(grid[0], grid[1], grid[2])
        state_grid = self.grids_info.get_state(id)
        path = [state_grid]
        pred = self.grids_info.get_predecessor(id)

        trace_time = 1
        while pred is not None:
            trace_time = trace_time + 1
            id = self.grids_info.get_idx(pred[0], pred[1], pred[2])
            state_pred = self.grids_info.get_state(id)
            path.insert(0, state_pred)
            #print(state_pred)
            pred = self.grids_info.get_predecessor(id)

            #if trace_time >100:
            #    print(trace_time)
            #    return path
        print(trace_time)
        return path



    def get_path(self, start_grid, goal_grid, grid_map):
        ''' Get path by traditional A* search.

        Arguments
        ---------
            start_grid (tuple): The start grid.

            goal_grid (tuple): The goal grid.

            grid_map (GridMap): The grid map.

        Returns
        -------
            _ (DrPath): The planned path.
        '''
        self.ready_for_search(start_grid, goal_grid, grid_map)
        self.iterations = 0

        # Search
        while len(self.open_list) != 0:
            # Pop the grid with minimum f cost.
            min_id = self.grids_info.get_min_id(self.open_list)
            grid = self.open_list.pop(min_id)
            id_grid = grid[2] *grid_map.max_x * grid_map.max_y + grid[1] * grid_map.max_x + grid[0]
            state_grid = self.grids_info.get_state(id_grid)
            self.iterations = self.iterations + 1

            print('present_grid:',grid,state_grid)


            if grid in self.close_list:
                continue
            else :
                # Push it into close list.
                self.close_list.append(grid)


            # If the goal grid is pushed into close list,
            # the path is found.
            if abs(state_grid[0] - goal_grid[0]) < 0.5 and abs(state_grid[1] - goal_grid[1]) < 0.5 and int(abs((state_grid[2] - goal_grid[2])/0.206)) < 3:
                # if id_goal == id_grid:
                print(self.iterations)
                print('total_cost:',self.grids_info.get_g_cost(id_grid))
                self.path = self.trace_back(grid)
                return self.path
            if self.iterations > self.iteration_max:
                print('Failed',self.iterations)
                #print(self.grids_info.get_predecessor(start_grid[0], start_grid[1], start_grid[2]))
                #print(grid)
                self.path = self.trace_back(grid)
                return self.path

            # For each possible control.
            for u in self.robot_model.get_controls(state_grid, grid_map):
                state_succ = (u[0], u[1], u[2])
                succ = (int(state_succ[0]), int(state_succ[1]), int(state_succ[2] /0.206))
                prim = u[3]
                id_succ = succ[2] *grid_map.max_x * grid_map.max_y+ succ[1] * grid_map.max_x+ succ[0]

                #print('succ_grid:',succ, state_succ)
                if id_succ > 198400:
                    continue

                if succ in self.close_list: #and id_grid != id_succ:
                    continue

                #If not in closed_list or in the same cell,calculate new_g_cost
                newG = self.grids_info.get_g_cost(id_grid) + self.get_transition_cost(state_grid, state_succ, grid_map, prim)
                #print('newG',newG)

                # If collision.
                if newG == float('inf'):
                    continue

                if succ in self.open_list and newG > self.grids_info.get_g_cost(id_succ):# and id_grid != id_succ:
                    continue

                #If not in open_list or in the same cell or have lower g_cost, calculate new_h_cost
                newH = self.get_heuristic(state_succ, goal_grid)
                #print('newH:',newH)
                #print('newF', newG+newH)

                if id_grid == id_succ and (newH + newG) > (self.grids_info.get_f_cost(id_grid) + 0.01):
                    continue

                # If looping
                pred = self.grids_info.get_predecessor(id_grid)
                if pred == succ:
                        continue

                # Update succ_grid's information
                #if id_grid == id_succ and (newH + g_cost) <= (self.grids_info.get_f_cost(id_grid) + 0.01):
                #    self.grids_info.set_predecessor(id_succ, pred)
                #    #print('lowerfcost')
                #else:
                #    self.grids_info.set_predecessor(id_succ, grid)

                self.grids_info.set_predecessor(id_succ, grid)

                self.grids_info.set_state(state_succ[0], state_succ[1], state_succ[2])
                self.grids_info.set_g_cost(id_succ, newG)
                self.grids_info.set_h_cost(id_succ, newH)
                if succ not in self.open_list:
                    self.open_list.append(succ)

                #print(succ,state_succ)
                #print('pred:', self.grids_info.get_predecessor(id_succ))
            print('-------')

        self.path.append(start_grid)
        return self.path



    def show_path(self, topic_name, grid_map):
        pub = rospy.Publisher(topic_name, Marker, queue_size=10, latch=True)

        # Build message for path.
        msg = Marker()
        msg.header.frame_id = '/map'
        msg.ns = 'a_star_planner'
        msg.pose.position.x = grid_map.resolution / 2.0 # Offset
        msg.pose.position.y = grid_map.resolution / 2.0 # Offset
        msg.type = msg.LINE_STRIP
        msg.action = msg.ADD
        msg.scale.x = 0.05
        #msg.scale.x = grid_map.resolution # Point width
        #msg.scale.y = grid_map.resolution # Point height
        for grid in self.path:
            x = grid[0] * grid_map.resolution - grid_map.resolution / 2.0
            y = (grid_map.max_y - 1 - grid[1]) * grid_map.resolution + grid_map.resolution / 2.0
            msg.points.append(Point(x=x, y=y))
            msg.colors.append(ColorRGBA(g=0.8, a=1.0))

        pub.publish(msg)



    def send_path(self, topic_name, grid_map):
        ''' Send self.path in the class as a nav_msgs/Path.msg.
        
        The unit of x and y is in grid.
        '''
        pub = rospy.Publisher(topic_name, Path, queue_size=10, latch=True)
        msg = Path()
        msg.header.frame_id = '/map'
        for grid in self.path:
            ps = PoseStamped()
            ps.pose.position.x = grid[0] * grid_map.resolution
            ps.pose.position.y = (grid_map.max_y - grid[1]) * grid_map.resolution
            msg.poses.append(ps)

        pub.publish(msg)

