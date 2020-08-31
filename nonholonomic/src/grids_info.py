import sys
import math
import numpy as np
from grid_info import GridInfo

class GridsInfo(object):
    ''' The matrix of grid information.

    Arguments
    ---------
        max_x: The width of matrix, from left to right.

        max_y: The height of matrix, from down to up.
    '''


    def __init__(self, max_x, max_y):
        self.max_x = max_x
        self.max_y = max_y
        self.mat = np.zeros((max_x * max_y * 31,), dtype=GridInfo)
        for t in range(31):
            for y in range(max_y):
                for x in range(max_x):
                    self.mat[t*max_x*max_y + y*max_x + x] = GridInfo()



    def get_min_id(self, open_list):
        '''
        Arguments
        ---------
            open_list (list): The element in list is grid (tuple).

        Returns
        -------
            min_id (int): The index of the grid with the minimum f cost.
                          Return -1 if open_list is empty.
        '''
        min_id = -1
        min_f_cost = sys.float_info.max
        for idx, grid in enumerate(open_list):
            f_cost = self.get_f_cost(self.get_idx(grid[0], grid[1], grid[2]))
            if f_cost < min_f_cost:
                min_id = idx
                min_f_cost = f_cost

        return min_id

    def set_state(self, x , y , t):
        idx = int(t / 0.206) * self.max_x * self.max_y + int(y) * self.max_x + int(x)
        self.mat[idx].idx = idx
        self.mat[idx].state = (x, y, t)


    def get_state(self, idx):
        return self.mat[idx].state


    def get_idx(self, x, y , t):
        idx = int(t) *self.max_x * self.max_y + int(y) * self.max_x + int(x)
        return self.mat[idx].idx



    def get_g_cost(self, idx):
        return self.mat[idx].g_cost



    def set_g_cost(self, idx, g_cost):
        self.mat[idx].g_cost = g_cost



    def get_h_cost(self, idx):
        return self.mat[idx].h_cost



    def set_h_cost(self, idx, h_cost):
        self.mat[idx].h_cost = h_cost



    def get_f_cost(self, idx):
        return self.mat[idx].g_cost + self.mat[idx].h_cost



    def get_predecessor(self, idx):
        '''
        Returns
        -------
            _ (tuple): The predecessor grid.
        '''
        #idx = int(t * 36 / math.pi) * self.max_x * self.max_y + int(y) * self.max_x + int(x)
        return self.mat[idx].predecessor



    def set_predecessor(self, idx, predecessor):
        '''
        Arguments
        ---------
        predecessor (tuple): The predecessor grid.
        '''
        self.mat[idx].predecessor = predecessor
