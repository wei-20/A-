import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
# from colorama import init as clr_ama_init
# from colorama import Fore
# clr_ama_init(autoreset = True)

class GridMap(object):
    ''' Grid map used for planning.

    Arguments
    ---------
        picture (np.array): Picture that describes the grid map.
        
        resolution (float): The size (meter) of one grid.
    '''



    def __init__(self, picture, resolution):
        # Check
        assert(resolution > 0.0)

        # Member
        self.max_x = picture.shape[1]
        self.max_y = picture.shape[0]
        self.resolution = resolution
        self.occupancies = np.zeros((self.max_x, self.max_y))
        
        # Set self.occupancies
        picture = 255 * picture
        picture = picture.astype(int)
        for x in range(self.max_x):
            for y in range(self.max_y):
                pixel_value = picture[self.max_y-1-y][x]
                if (pixel_value == 0).any():
                    self.occupancies[x][y] = 1.0 # occupied
                elif (pixel_value == 127).any():
                    self.occupancies[x][y] = -0.01 # unknown
                else:
                    self.occupancies[x][y] = 0.0 # free



    def occupancy(self, x, y, time=0.0):
        ''' Return the occupancy of grid at (x, y) of specific time.

        Arguments
        ---------
            x (int): From left to right.

            y (int): From down to up.

            time (float): 0.0 (seconds) represents the current time.

        Returns
        -------
            occ (float): The occupancy.
        '''
        x = np.clip(x, 0, self.max_x-1)
        y = np.clip(y, 0, self.max_y-1)
        occ = self.occupancies[x][y]
        return occ



    def grid_type(self, x, y, time=0.0):
        occ = self.occupancy(x, y)
        if occ < 0.0:
            tp = 'unknown'
        elif occ < 0.25:
            tp = 'free'
        elif occ > 0.75:
            tp = 'occupied'
        else:
            tp = 'gray'

        return tp



    def show(self, topic_name, time=0.0):
        ''' Show the grid map of specific time in RViz.

        Arguments
        ---------
            topic_name (str): Topic name.
            
            time (float): 0.0 (seconds) represents the current time.
        '''
        pub = rospy.Publisher(topic_name, OccupancyGrid, queue_size=10, latch=True)
        msg = OccupancyGrid()
        msg.info.resolution = self.resolution
        msg.info.width = self.max_x
        msg.info.height = self.max_y
        for y in range(self.max_y-1, -1, -1):
            for x in range(self.max_x):
                msg.data.append(int(self.occupancies[x][y] * 100))

        pub.publish(msg)



    # def print(self):
    #     ''' Print infomation about the grid map.
    #     '''
    #     print('Grid map information\n--------------------\nmax_x: {}\nmax_y: {}\nresolution: {:.3f}\n'.format(self.max_x,
    #         self.max_y, self.resolution))



    def set_grid(self, x, y, occ):
        x = np.clip(x, 0, self.max_x-1)
        y = np.clip(y, 0, self.max_y-1)
        self.occupancies[x][y] = occ
