import sys

class GridInfo(object):
    ''' The infomation about grid in A* search.

    Members
    -------
        g_cost: The g cost of grid.
        
        h_cost: The h (heuristic) cost of grid.

        state: The continuous state associated with grid.

        predecessor (tuple): The predecessor grid of this grid.
    '''



    def __init__(self):
        # Member
        self.idx = 0
        self.g_cost = sys.float_info.max
        self.h_cost = 0.0
        self.state = None
        self.predecessor = None



    # def print(self):
    #     print('Grid information\n----------------\nf cost: {:.3f}\n'.format(self.g_cost + self.h_cost))
