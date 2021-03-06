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
    def __init__(self, maze_dim, sufficient_visit_threshold=0.8):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''

        # maze dimension
        self.maze_dim = maze_dim

        # tuning parameter: sufficient visit threshold
        self.sufficient_visit_threshold = sufficient_visit_threshold


        self.prepare_1st_run()


    def prepare_1st_run(self):
        '''Consolidate all initialization for 1st run.'''

        maze_dim = self.maze_dim

        # initial location
        self.location = [0, 0]

        # initial heading
        self.heading = 'u'

        # record time steps for each run
        self.time = 0

        # bool indicates whether to explore the maze (1st run) or navigate to the goal (2nd run)
        self.is_to_explore = True

        '''
        Heuristic matrix looks like this
        [[5 5 5 5 5 5 5 5 5 5 5 5]
         [5 4 4 4 4 4 4 4 4 4 4 5]
         [5 4 3 3 3 3 3 3 3 3 4 5]
         [5 4 3 2 2 2 2 2 2 3 4 5]
         [5 4 3 2 1 1 1 1 2 3 4 5]
         [5 4 3 2 1 0 0 1 2 3 4 5]
         [5 4 3 2 1 0 0 1 2 3 4 5]
         [5 4 3 2 1 1 1 1 2 3 4 5]
         [5 4 3 2 2 2 2 2 2 3 4 5]
         [5 4 3 3 3 3 3 3 3 3 4 5]
         [5 4 4 4 4 4 4 4 4 4 4 5]
         [5 5 5 5 5 5 5 5 5 5 5 5]]
        '''
        heuristic = np.full((maze_dim, maze_dim), 0, dtype=int)
        center_n = float(maze_dim-1)/2
        for i in range(maze_dim):
            for j in range(maze_dim):
                i_abs = abs(i-center_n)
                j_abs = abs(j-center_n)
                heuristic[i][j] = int(max(i_abs, j_abs))
        self.heuristic = heuristic

        inv_heuristic = np.full((maze_dim, maze_dim), 0, dtype=int)
        center_n = float(maze_dim-1)/2
        for i in range(maze_dim):
            for j in range(maze_dim):
                i_abs = abs(i-center_n)
                j_abs = abs(j-center_n)
                inv_heuristic[i][j] = maze_dim/2-int(max(i_abs, j_abs))-1
        self.inv_heuristic = inv_heuristic

        # penalty multiplier for visiting visited cells
        self.visited_cell_penalty = maze_dim/2 - 1

        '''
        To record how many times each cell has being passed by robot
        during exploration.

        This matrix will be populated during 1st run, and retained for 2nd run
        as boundary check.

        Cells not visited at 1st run will be 'closed' down for 2nd run.
        '''
        pathCounts = np.full((maze_dim, maze_dim), 0, dtype=int)
        pathCounts[tuple(self.location)] = 1
        self.pathCounts = pathCounts

        '''
        Wall info matrix: 1 is open, 0 is closed.
        '''
        walls = [[{'u':1,'r':1,'d':1,'l':1} for i in range(maze_dim)] for j in range(maze_dim)]
        self.walls = np.array(walls)
        self.walls[tuple(self.location)] = {'u':1,'r':0,'d':0,'l':0}

        # record path sequence for 1st run
        self.paths = np.full((self.maze_dim, self.maze_dim), 0, dtype=int)

        # if once visited goal, set to True
        self.visited_goal = False


    def prepare_2nd_run(self):
        '''Consolidate all initialization for 2nd run.'''

        print "Path sequences:"
        print np.rot90(self.paths)
        print "Visit counts:"
        print np.rot90(self.pathCounts)
        print "Exploration time steps: {}".format(self.time)

        self.location = [0, 0]
        self.heading = 'u'
        self.time = 0
        self.is_to_explore = False

        # record path sequence for 2nd run
        self.paths = np.full((self.maze_dim, self.maze_dim), 0, dtype=int)

    def has_sufficient_map_info(self):
        '''
        Check if robot has sufficient map info by calculating
        ratio = visited cells / totoal cells
        '''
        visited_cells = 0
        n = self.maze_dim
        for i in range(n):
            for j in range(n):
                if self.pathCounts[i][j] > 0:
                    visited_cells += 1

        covered = float(visited_cells) /(n*n)
        print "covered {0:.1f} %".format(covered * 100)
        return covered > self.sufficient_visit_threshold

    def can_transit(self, side, from_loc=None, dist=1):
        '''
        Check if robot can transit from one location (`from_loc`) to another via a specific `side`.
        Default from-location is `self.location`.
        Default distance (`dist`) to transit is 1.
        '''

        if from_loc is None:
            from_loc = tuple(self.location)

        assert isinstance(from_loc, tuple)
        assert dist in (1,2,3)

        max_dist = dist
        can = (self.walls[from_loc][side] == 1)
        if can:
            for d in range(1,max_dist+1):
                delta_i = dir_move[side][0] * d
                delta_j = dir_move[side][1] * d
                to_loc = (from_loc[0]+delta_i, from_loc[1]+delta_j)
                if self.is_inside_maze(to_loc):
                    wall = self.walls[to_loc]

                    # every cell on that side within dist should open opposite side
                    can = can and (wall[dir_reverse[side]]==1)
                    if d != max_dist:
                        # except farthest cell, all other cells on that side within dist should open the same side
                        can = can and (wall[side]==1)

                    if not can:
                        # as soon as one cell has wall blocking the way, stop checking
                        break
                else:
                    # if a cell on that side within dist is out of maze, stop checking
                    can = False
                    break

        return can

    def compute_position_for_transit(self, side, from_loc=None, from_head=None, dist=1):
        '''
        Compute robot state (location and heading) after a transition.
        Default from-location is `self.location`.
        Default from-heading is `self.heading`.
        Default distance (`dist`) to transit is 1.
        '''
        if from_loc is None:
            from_loc = tuple(self.location)

        if from_head is None:
            from_head = self.heading

        assert isinstance(from_loc, tuple)
        assert isinstance(from_head, str)
        assert dist in (1,2,3)


        new_loc = (from_loc[0] + dir_move[side][0]*dist, from_loc[1] + dir_move[side][1]*dist)
        new_head = None

        if side == dir_reverse[from_head]:
            # need to reverse, heading remains the same
            new_head = from_head
        else:
            # go straight, turn left or turn right, new heading aligns with side
            new_head = side

        return new_loc, new_head

    def compute_motion_for_transit(self, side, from_head=None, dist=1):
        '''
        Compute the motion (rotation and movement) needed to transit from current heading via chosen side.
        Default from-heading is `self.heading`.
        Default distance (`dist`) to transit is 1.
        '''
        if from_head is None:
            from_head = self.heading

        assert isinstance(from_head, str)
        assert dist in (1,2,3)

        heading_i = wall_sides.index(from_head)
        side_i = wall_sides.index(side)
        if heading_i == side_i:
            return 0, dist
        elif (heading_i - 1) % 4 == side_i:
            return -90, dist
        elif (heading_i + 1) % 4 == side_i:
            return 90, dist
        elif abs(heading_i - side_i) == 2:
            return 0, -dist
        else:
            raise "Invalid heading {} and side {}".format(from_head, side)

    def transit(self, rotation, movement):
        '''
        Make actual changes to robot's state (location and heading).

        The logic is taken from tester.py to maintain robot state consistency.
        '''

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
        '''
        Update walls matrix using distance data from sensor data.
        '''

        # get actual sides in cell's coordinate from robot heading
        sides = dir_sensors[self.heading]

        for i in range(3):
            side = sides[i]
            dist = sensors[i]
            loc = list(self.location)
            while dist > 0:
                cell = self.walls[tuple(loc)]
                cell[side] = 1 # no wall on that side
                if loc == self.location:
                    # if it is current cell, no info on opposite side
                    pass
                else:
                    # has no wall on opposite side
                    cell[dir_reverse[side]] = 1
                loc[0] += dir_move[side][0]
                loc[1] += dir_move[side][1]
                dist -= 1

            # this is the cell with the wall that sensor hits on
            # could be current cell when dist == 0
            # could be another cell when dist > 0
            walled_cell = self.walls[tuple(loc)]
            walled_cell[side] = 0 # has wall on that side
            if loc == self.location:
                # if it is current cell, no info on opposite side
                pass
            else:
                # has no wall on opposite side
                walled_cell[dir_reverse[side]] = 1

    def get_maze(self):
        '''
        Tranform walls matrix into maze matrix containing the same numbers as in the input data file.

        Useful for debugging.
        '''
        n = self.maze_dim
        maze = np.zeros((n,n),dtype=int)
        for i in range(n):
            for j in range(n):
                wall = self.walls[i][j]
                maze[i][j] = wall['u']*1 + wall['r']*2 + wall['d']*4 + wall['l']*8

        return maze

    def is_goal(self,location=None):
        '''
        Check if a location is within goal area.

        Default location is robot's current location.
        '''

        if location is None:
            location = self.location

        goal_bounds = [self.maze_dim/2 - 1, self.maze_dim/2]
        return (location[0] in goal_bounds and location[1] in goal_bounds)

    def is_inside_maze(self,location):
        '''
        Check if a location is inside a maze or not.
        '''
        assert isinstance(location, tuple)

        x_inside = (location[0]>=0 and location[0]<self.maze_dim)
        y_inside = (location[1]>=0 and location[1]<self.maze_dim)

        return (x_inside and y_inside)

    def search_for_cost(self):
        '''
        Use A* search algorithm to explore the maze and find the goal.
        '''

        assert self.is_to_explore

        n = self.maze_dim
        step_cost = 1
        start_loc = tuple(self.location)
        start_head = self.heading

        visited = np.zeros((n,n), dtype=int)
        visited[start_loc] = 1

        max_int = np.iinfo(np.int16).max
        costs = np.full((n,n), max_int, dtype=int)
        costs[start_loc] = 0

        g = 0
        h = self.heuristic[start_loc]
        f = g + h

        openlist = [[f, g, h, start_loc, start_head]]

        found = False
        while not found:
            if len(openlist) == 0:
                raise "Error: openlist becomes emtpy!"

            else:
                openlist.sort(reverse=True)
                next = openlist.pop()
                f = next[0]
                g = next[1]
                h = next[2]
                loc = next[3]
                head = next[4]

                costs[loc] = f

                if self.is_goal(loc):
                    found = True
                else:
                    for side in wall_sides:
                        if self.can_transit(side,from_loc=loc):
                            loc2, head2 = self.compute_position_for_transit(side,from_loc=loc,from_head=head)
                            if self.is_inside_maze(loc2):
                                # when explore, append loc2 if it has NOT been searched for cost
                                if visited[loc2] == 0:
                                    g2 = g + step_cost + self.pathCounts[loc2] * self.visited_cell_penalty
                                    # invert heuristic when continue searching after reaching goal
                                    h2 = self.inv_heuristic[loc2] if self.visited_goal else self.heuristic[loc2]
                                    f2 = g2 + h2
                                    openlist.append([f2, g2, h2, loc2, head2])
                                    visited[loc2] = 1




        return costs

    def search_for_policy(self):
        '''
        Use dynamic programming to find optimal policy based on map data gathered in 1st run.
        '''

        assert not self.is_to_explore

        step_value = 1
        n = self.maze_dim
        n_sides = len(wall_sides)
        values = np.array([[[999 for i in range(n)] for j in range(n)] for k in range(n_sides)])
        policy = np.array([[[(0,0) for i in range(n)] for j in range(n)] for k in range(n_sides)], dtype=tuple)
        change = True
        while change:
            change = False

            for i in range(n):
                for j in range(n):
                    for h in range(n_sides):
                        head = wall_sides[h]
                        loc = (i, j)
                        if self.is_goal(loc):
                            if values[h][i][j] > 0:
                                values[h][i][j] = 0
                                change = True

                        elif self.pathCounts[loc] > 0:
                            for side in wall_sides:
                                for dist in (1,2,3):
                                    if self.can_transit(side,from_loc=loc,dist=dist):
                                        loc2, head2 = self.compute_position_for_transit(side,from_loc=loc,from_head=head,dist=dist)
                                        (rot2, mov2) = self.compute_motion_for_transit(side,from_head=head,dist=dist)
                                        if self.is_inside_maze(loc2) and self.pathCounts[loc2] > 0:
                                            (i2, j2) = loc2
                                            h2 = wall_sides.index(head2)
                                            v2 = values[h2][i2][j2] + step_value
                                            if v2 < values[h][i][j]:
                                                values[h][i][j] = v2
                                                policy[h][i][j] = (rot2, mov2)
                                                change = True





        self.policy = policy

    def explore_min_cost(self):
        '''
        Make a move based with minimum cost to explore.

        Every move is made based on current belief of the maze.
        '''

        # cost matrix based on current belief
        costs = self.search_for_cost()

        # sides that robot can transit through
        sides = []
        for side in wall_sides:
            if self.can_transit(side):
                sides.append(side)

        side_costs = []
        for side in sides:
            loc, _ = self.compute_position_for_transit(side)
            side_costs.append(costs[loc])

        min_cost = min(side_costs)
        # find sides indices that have minimum cost
        min_cost_i = [i for i, v in enumerate(side_costs) if v == min_cost]
        # if there are more than one sides, randomly choose one
        side = sides[random.choice(min_cost_i)]

        r, m = self.compute_motion_for_transit(side)
        self.transit(r, m)
        self.paths[tuple(self.location)] = self.time
        self.pathCounts[tuple(self.location)] += 1
        return r, m


    def random_explore(self):
        '''
        Randomly explore without cost calculation.
        '''
        r = 0
        m = 0
        while r is None and m is None:
            side = random.choice(wall_sides)
            if self.can_transit(side):
                r, m = self.compute_motion_for_transit(side)

        self.transit(r, m)
        self.paths[tuple(self.location)] = self.time
        self.pathCounts[tuple(self.location)] += 1
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


        rotation = 0
        movement = 0
        if self.is_to_explore:
            if self.visited_goal and self.has_sufficient_map_info():
                rotation = 'Reset'
                movement = 'Reset'
                self.prepare_2nd_run()
                self.search_for_policy()
            else:
                self.time += 1
                print ">>> Time {}".format(self.time)
                self.update_walls(sensors)
                # rotation, movement = self.random_explore()
                rotation, movement = self.explore_min_cost()
                print "current loc: {} heading: {}".format(self.location, self.heading)
                print "next move r: {} m: {}".format(rotation, movement)
                if self.is_goal():
                    # goal location found by 1st run
                    print "Goal is found at: {}".format(tuple(self.location))
                    self.visited_goal = True
        else:
            self.time += 1
            print ">>> Time {}".format(self.time)
            i = self.location[0]
            j = self.location[1]
            h = wall_sides.index(self.heading)
            (rotation, movement) = self.policy[h][i][j]
            self.transit(rotation, movement)
            print "    At location:({},{}) -> policy:({},{})".format(i, j, rotation, movement)
            self.paths[tuple(self.location)] = self.time
            if self.is_goal(tuple(self.location)):
                print "Path sequences:"
                print np.rot90(self.paths)
                print "Navigation time steps: {}".format(self.time)


        return rotation, movement
