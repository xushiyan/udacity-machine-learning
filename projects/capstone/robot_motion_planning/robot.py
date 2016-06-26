import numpy as np
import random

# global dictionaries for robot movement and sensing
dir_sensors = {'u': ['l', 'u', 'r'], 'r': ['u', 'r', 'd'],
               'd': ['r', 'd', 'l'], 'l': ['d', 'l', 'u'],
               'up': ['l', 'u', 'r'], 'right': ['u', 'r', 'd'],
               'down': ['r', 'd', 'l'], 'left': ['d', 'l', 'u']}
dir_move = {'u': [0, 1], 'r': [1, 0], 'd': [0, -1], 'l': [-1, 0],
            'up': [0, 1], 'right': [1, 0], 'down': [0, -1], 'left': [-1, 0]}
dir_reverse = {'u': 'd', 'r': 'l', 'd': 'u', 'l': 'r',
               'up': 'd', 'right': 'l', 'down': 'u', 'left': 'r'}

wall_sides = ('u', 'r', 'd', 'l')


class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        # make sure maze_dim is set first
        self.maze_dim = maze_dim

        self.reset()

        # bool indicates whether to explore the maze or navigate to the goal
        self.is_to_explore = True

        # wall info: 1 is open, 0 is closed
        walls = [[{'u':1,'r':1,'d':1,'l':1} for i in range(maze_dim)] for j in range(maze_dim)]
        self.walls = np.array(walls)
        self.walls[tuple(self.location)] = {'u':1,'r':0,'d':0,'l':0}


    def reset(self):
        self.location = [0, 0]
        self.heading = 'u'
        self.time = 0

        n = self.maze_dim

        # path: step squence that robot has taken
        paths = np.full((n, n), -1, dtype=int)
        paths[tuple(self.location)] = 0
        self.paths = paths

        # path count: how many times the same cell being passed
        pathCounts = np.full((n, n), 0, dtype=int)
        pathCounts[tuple(self.location)] = 1
        self.pathCounts = pathCounts

    def can_transit(self, side, from_loc=None):
        if from_loc is None:
            from_loc = tuple(self.location)


        assert isinstance(from_loc, tuple)

        wall = self.walls[from_loc]
        return wall[side] == 1

    def compute_position_for_transit(self, side, from_loc=None, from_head=None):
        if from_loc is None:
            from_loc = tuple(self.location)

        if from_head is None:
            from_head = self.heading

        assert isinstance(from_loc, tuple)
        assert isinstance(from_head, str)

        new_loc = (from_loc[0] + dir_move[side][0], from_loc[1] + dir_move[side][1])
        new_head = None

        if side == dir_reverse[from_head]:
            # need to reverse, heading remains the same
            new_head = from_head
        else:
            # go straight, turn left or turn right, new heading aligns with side
            new_head = side

        return new_loc, new_head

    def compute_motion_for_transit(self, side, from_head=None):
        if from_head is None:
            from_head = self.heading

        assert isinstance(from_head, str)

        heading_i = wall_sides.index(from_head)
        side_i = wall_sides.index(side)
        if heading_i == side_i:
            return 0, 1
        elif (heading_i - 1) % 4 == side_i:
            return -90, 1
        elif (heading_i + 1) % 4 == side_i:
            return 90, 1
        elif abs(heading_i - side_i) == 2:
            return 0, -1
        else:
            raise "Invalid heading {} and side {}".format(from_head, side)

    def transit(self, rotation, movement):
        # perform rotation
        if rotation == -90:
            self.heading = dir_sensors[self.heading][0]
        elif rotation == 90:
            self.heading = dir_sensors[self.heading][2]
        elif rotation == 0:
            pass
        else:
            print "Invalid rotation value, no rotation performed."

        # perform movement
        if abs(movement) > 3:
            print "Movement limited to three squares in a turn."
        movement = max(min(int(movement), 3), -3) # fix to range [-3, 3]
        while movement:
            if movement > 0:
                if self.can_transit(self.heading):
                    self.location[0] += dir_move[self.heading][0]
                    self.location[1] += dir_move[self.heading][1]
                    movement -= 1
                else:
                    print "Movement stopped by wall."
                    movement = 0
            else:
                rev_heading = dir_reverse[self.heading]
                if self.can_transit(rev_heading):
                    self.location[0] += dir_move[rev_heading][0]
                    self.location[1] += dir_move[rev_heading][1]
                    movement += 1
                else:
                    print "Movement stopped by wall."
                    movement = 0

    def update_walls(self, sensors):
        # print "sensors: {}".format(sensors)
        current_cell = self.walls[tuple(self.location)]
        sides = dir_sensors[self.heading]

        for i in range(3):
            side = sides[i]
            dist = sensors[i]
            loc = list(self.location)
            while dist > 0:
                cell = self.walls[tuple(loc)]
                cell[side] = 1 # side open
                if loc != self.location:
                    cell[dir_reverse[side]] = 1 # side open
                loc[0] += dir_move[side][0]
                loc[1] += dir_move[side][1]
                dist -= 1

            walled_cell = self.walls[tuple(loc)]
            walled_cell[side] = 0 # side closed
            if loc != self.location:
                walled_cell[dir_reverse[side]] = 1 # side open

    def get_maze(self):
        n = len(self.walls)
        maze = np.zeros((n,n),dtype=int)
        for i in range(n):
            for j in range(n):
                wall = self.walls[i][j]
                maze[i][j] = wall['u']*1 + wall['r']*2 + wall['d']*4 + wall['l']*8

        return maze

    def is_goal(self,location=None):
        if location is None:
            location = self.location

        dim = len(self.walls)
        goal_bounds = [dim/2 - 1, dim/2]
        return (location[0] in goal_bounds and location[1] in goal_bounds)

    def is_inside_maze(self,location):

        assert isinstance(location, tuple)

        dim = len(self.walls)
        x_inside = location[0]>=0 and location[0]<dim
        y_inside = location[1]>=0 and location[1]<dim
        return (x_inside and y_inside)

    def search_cost(self):

        n = len(self.walls)
        step_cost = 1
        past_path_cost = 2
        start_loc = tuple(self.location)
        start_head = self.heading

        visited = np.zeros((n,n), dtype=int)
        visited[start_loc] = 1

        max_int = np.iinfo(np.int16).max
        costs = np.full((n,n), max_int, dtype=int)
        costs[start_loc] = 0

        g = 0

        openlist = [[g, start_loc, start_head]]

        found = False
        while not found:
            if len(openlist) == 0:
                raise "Error: openlist becomes emtpy!"

            else:
                openlist.sort(reverse=True)
                next = openlist.pop()
                g = next[0]
                loc = next[1]
                head = next[2]

                costs[loc] = g

                if self.is_goal(loc):
                    found = True
                else:
                    for side in wall_sides:
                        if self.can_transit(side,from_loc=loc):
                            loc2, head2 = self.compute_position_for_transit(side,from_loc=loc,from_head=head)
                            (r1_2, m1_2) = self.compute_motion_for_transit(side,from_head=head)
                            if self.is_inside_maze(loc2):
                                if visited[loc2] == 0: # loc2 has NOT been searched for cost
                                    # if self.paths[loc2] >= 0: # loc2 has been visited in actual path
                                    #     g2 = g + past_path_cost
                                    # elif m1_2 < 0: # need to revese
                                    #     g2 = g + rev_cost
                                    # else:
                                        # g2 = g + step_cost
                                    g2 = g + step_cost + self.pathCounts[loc2] * past_path_cost

                                    openlist.append([g2, loc2, head2])
                                    visited[loc2] = 1




        return costs

    def explore_min_cost(self):

        costs = self.search_cost()

        sides = []
        for side in wall_sides:
            if self.can_transit(side):
                sides.append(side)

        side_costs = []
        for side in sides:
            loc, _ = self.compute_position_for_transit(side)
            side_costs.append(costs[loc])

        min_cost = min(side_costs)
        min_cost_i = [i for i, v in enumerate(side_costs) if v == min_cost]

        side = sides[random.choice(min_cost_i)]

        r, m = self.compute_motion_for_transit(side)
        self.transit(r, m)
        self.paths[tuple(self.location)] = self.time
        self.pathCounts[tuple(self.location)] += 1
        return r, m


    def random_explore(self):

        r = None
        m = None
        while r is None and m is None:
            side = random.choice(wall_sides)
            if self.can_transit(side):
                r, m = self.compute_motion_for_transit(side)

        self.transit(r, m)

        return r, m


    def next_move(self, sensors):
        '''
        Use this function to determine the next move the robot should make,
        based on the input from the sensors after its previous move. Sensor
        inputs are a list of three distances from the robot's left, front, and
        right-facing sensors, in that order.

        Outputs should be a tuple of two values. The first value indicates
        robot rotation (if any), as a number: 0 for no rotation, +90 for a
        90-degree rotation clockwise, and -90 for a 90-degree rotation
        counterclockwise. Other values will result in no rotation. The second
        value indicates robot movement, and the robot will attempt to move the
        number of indicated squares: a positive number indicates forwards
        movement, while a negative number indicates backwards movement. The
        robot may move a maximum of three units per turn. Any excess movement
        is ignored.

        If the robot wants to end a run (e.g. during the first training run in
        the maze) then returing the tuple ('Reset', 'Reset') will indicate to
        the tester to end the run and return the robot to the start.
        '''
        print "==========================="
        self.time += 1
        rotation = 0
        movement = 0
        if self.is_to_explore:
            if self.is_goal():
                rotation = 'Reset'
                movement = 'Reset'
                self.reset()
                self.is_to_explore = False
            else:
                self.update_walls(sensors)
                rotation, movement = self.explore_min_cost()
                print np.rot90(self.paths)
                print "current loc: {} heading: {}".format(self.location, self.heading)
                print "next move\tr: {} m: {}".format(rotation, movement)
        else:
            pass


        print "time {}".format(self.time)


        return rotation, movement
