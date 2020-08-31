import math

class RobotModel(object):



    def __init__(self, radius):
        # Member
        self.radius = radius



    def get_controls(self, grid, grid_map):
        '''
        Returns
        -------
            controls (list): Each control is a tuple.
        '''
        if grid[0] < 0 or grid[0] > grid_map.max_x - 1 or grid[1] < 0 or grid[1] > grid_map.max_y - 1:
            return []

        dt = [0, 0.2292, -0.2292]
        #dt = [0, math.pi / 36, -math.pi / 36]
        #dt = [0, math.pi/36, -math.pi/36, math.pi/18, -math.pi/18]

        # turning angle = 0.206 num=31 198400 v=1.413716 r=6.85 dt=math.pi/36 5du
        # turning angle = 0.153 num=42 268800 v=1.05 r=6.85
        # 0.206 31 198400 1.413716 b=1.6 dt=0.2292 13.13du
        v = 1.413716
        b = 1.6

        controls = []
        for i in range(5):
            if i < 3:
                ta = v / b * math.tan(dt[i])
                x = grid[0] + v * math.cos(grid[2] + ta)
                y = grid[1] + v * math.sin(grid[2] + ta)
                t = grid[2] + ta
                # backward
            else:
                ta = v / b * math.tan(dt[i - 3])
                x = grid[0] - v * math.cos(grid[2] + ta)
                y = grid[1] - v * math.sin(grid[2] + ta)
                t = grid[2] + ta + math.pi

            if x < 0 or x > grid_map.max_x - 1 or y < 0 or y > grid_map.max_y - 1:
                continue

            if t < 0:
                t = t + 2*math.pi - 2*math.pi * int(t / (2.0*math.pi))
            else:
                t = t - 2*math.pi * int(t / (2.0*math.pi))

            if int(t /0.206) < 0 or int(t /0.206 ) > 31:
                continue

            #print(x,y,t)

            controls.append((x, y, t, i))
        #for dx in range(-1, 2):
        #    for dy in range(-1, 2):
        #        if dx == 0 and dy == 0:
        #            continue

        #        x = grid[0] + dx
        #        y = grid[1] + dy
        #        if x < 0 or x > grid_map.max_x - 1 or y < 0 or y > grid_map.max_y - 1:
        #            continue



        return controls
