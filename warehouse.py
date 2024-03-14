
import copy
import math

# If you see different scores locally and on Gradescope this may be an indication
# that you are uploading a different file than the one you are executing locally.
# If this local ID doesn't match the ID on Gradescope then you uploaded a different file.
OUTPUT_UNIQUE_FILE_ID = False
if OUTPUT_UNIQUE_FILE_ID:
    import hashlib, pathlib

    file_hash = hashlib.md5(pathlib.Path(__file__).read_bytes()).hexdigest()
    print(f'Unique file ID: {file_hash}')


class DeliveryPlanner_PartA:
    """
    Required methods in this class are:
    
      plan_delivery(self, debug = False) which is stubbed out below.  
        You may not change the method signature as it will be called directly 
        by the autograder but you may modify the internals as needed.
    
      __init__: which is required to initialize the class.  Starter code is 
        provided that initializes class variables based on the definitions in
        testing_suite_partA.py.  You may choose to use this starter code
        or modify and replace it based on your own solution
    
    The following methods are starter code you may use for part A.  
    However, they are not required and can be replaced with your
    own methods.
    
      _set_initial_state_from(self, warehouse): creates structures based on
          the warehouse and todo definitions and initializes the robot
          location in the warehouse
    
      _search(self, debug=False): Where the bulk of the A* search algorithm
          could reside.  It should find an optimal path from the robot
          location to a goal.  Hint:  you may want to structure this based
          on whether looking for a box or delivering a box.
  
    """

    ## Definitions taken from testing_suite_partA.py
    ORTHOGONAL_MOVE_COST = 2
    DIAGONAL_MOVE_COST = 3
    BOX_LIFT_COST = 4
    BOX_DOWN_COST = 2
    ILLEGAL_MOVE_PENALTY = 100

    def __init__(self, warehouse, todo):

        self.todo = todo
        self.boxes_delivered = []
        self.total_cost = 0
        self._set_initial_state_from(warehouse)
        self.grid = []
        self.delta = [[-1, 0],  # north
                      [0, -1],  # west
                      [1, 0],  # south
                      [0, 1],  # east
                      [-1, -1],  # northwest (diag)
                      [-1, 1],  # northeast (diag)
                      [1, 1],  # southeast (diag)
                      [1, -1]]  # southwest (diag)
        self.delta_directions = ["n", "w", "s", "e", "nw", "ne", "se", "sw"]
        self.delta_directions_reverse = ["s", "e", "n", "w", "se", "sw", "nw", "ne"]

        # Can use this for a visual debug
        # self.delta_name = ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # You may choose to use arrows instead
        self.delta_name = ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']

        # Costs for each move
        self.delta_cost = [self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST]

    ## state parsing and initialization function from testing_suite_partA.py
    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse(list(list)): the warehouse map.
        """
        rows = len(warehouse)
        cols = len(warehouse[0])

        self.warehouse_state = [[None for j in range(cols)] for i in range(rows)]
        self.dropzone = None
        self.boxes = dict()

        for i in range(rows):
            for j in range(cols):
                this_square = warehouse[i][j]

                if this_square == '.':
                    self.warehouse_state[i][j] = '.'

                elif this_square == '#':
                    self.warehouse_state[i][j] = '#'

                elif this_square == '@':
                    self.warehouse_state[i][j] = '*'
                    self.dropzone = (i, j)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)

        self.robot_position = self.dropzone
        self.box_held = None
    def locate(self, target):
        """
        :param target:the character representing the item you want to locate on the grid. Example: *, @, 1, a, f
        :return: a coordinate of that target [i,j]
        """
        nlist = [[i, el.index(target)] for i, el in enumerate(self.grid) if target in el]
        if nlist != []:
            res = nlist[0]
        else:
            res = []
        return res

    def identify(self,coordinate):
        """

        :param coordinate:a coordinate [i, j]
        :return: the value of the cell with coordinate [i,j] on the grid
        """
        return self.grid[coordinate[0]][coordinate[1]]


    def locate_adj(self,target):
        """

        :param target: the character representing the item for which you want to locate all its traversable adjacent cells.
        Example: *, @, 1, a, f
        :return: a list of all traversable adjacent cells coordinate [(0,0),(2,2),...]
        """

        target_pos = self.locate(target)
        target_adj = []
        for i in range(len(self.delta)):
            x = target_pos[0] + self.delta[i][0]
            y = target_pos[1] + self.delta[i][1]
            if 0 <= x < len(self.grid) and 0 <= y < len(self.grid[0]):
                if self.grid[x][y] != "#":
                    target_adj.append([x, y])
        return target_adj

    def _search(self, task, item, debug=False):
        """
        This method should be based on lesson modules for A*, see Search, Section 12-14.
        The bulk of the search logic should reside here, should you choose to use this starter code.
        Please condition any printout on the debug flag provided in the argument.  
        You may change this function signature (i.e. add arguments) as 
        necessary, except for the debug argument which must remain with a default of False
        :param start: coordinate [i, j] of the starting cell
                goal:  coordinate [i, j] of the goal cell
                action: 'pick up' or 'drop'
        :return a list of moves from start cell to goal cell
        """

        # get a shortcut variable for the warehouse (note this is just a view no copying)
        # print('self.warehouse_state')
        # for i in range(len(self.warehouse_state)):
        #     print(self.warehouse_state[i])
        # print('----------------------------------------')
        # ----------------------------------------
        # insert code here
        # ----------------------------------------
        if task == 'pick up':
            init = self.locate("*")
            goal = self.locate(item)
        elif task == 'drop':
            init = self.locate("*")
            if self.locate("@") == []:
                goal = self.locate("*")
            else:
                goal = self.locate("@")

        # open list element are of the type: [g, x, y]
        closed = [[0 for row in range(len(self.grid[0]))] for col in range(len(self.grid))]
        closed[init[0]][init[1]] = 1
        # print('closed[i]')
        # for i in range(len(closed)):
        #     print('      ', closed[i])


        expand = [[-1 for row in range(len(self.grid[0]))] for col in range(len(self.grid))]
        # print('expand[i]')
        # for i in range(len(expand)):
        #     print('      ', expand[i])

        action = [[-1 for row in range(len(self.grid[0]))] for col in range(len(self.grid))]
        # print('action[i]')
        # for i in range(len(action)):
        #     print('      ', action[i])

        x = init[0]
        y = init[1]
        g = 0

        open = [[g, x, y]]
        # print('open')
        # print(open)

        found = False  # Flag that is set when search complete
        resign = False  # Flag set if we can't find expand
        count = 0

        while found is False and resign is False:
            # check if we still have elements on the open list
            if len(open) == 0:
                resign = True
                # print('fail')
                # print('Search terminates without success')
            else:
                # remove node from list
                open.sort()
                open.reverse()
                next = open.pop()
                # print('Take first item')
                # print('Next')
                x = next[1]
                y = next[2]
                g = next[0]
                expand[x][y] = count
                count += 1
            # check if we are done
            # print('goal: ',goal)
            if x == goal[0] and y == goal[1]:
                found = True
                # print(next)
                # print('Search successful')
            else:
                # expand winning element and add to new open list
                for i in range(len(self.delta)):
                    x2 = x + self.delta[i][0]
                    y2 = y + self.delta[i][1]
                    if x2 >= 0 and x2 < len(self.grid) and y2 >= 0 and y2 < len(self.grid[0]):
                        if closed[x2][y2] == 0 and self.grid[x2][y2].lower() in ('.', '@', item.lower()):
                            g2 = g + self.delta_cost[i]
                            open.append([g2, x2, y2])
                            closed[x2][y2] = 1
                            action[x2][y2] = i
        # for i in range(len(expand)):
        #     print(expand[i])
        policy = [[' ' for col in range(len(self.grid[0]))] for row in range(len(self.grid))]
        direction = [[' ' for col in range(len(self.grid[0]))] for row in range(len(self.grid))]
        x = goal[0]
        y = goal[1]
        # policy[x][y] = '*'

        moves = []
        while x != init[0] or y != init[1]:
            x2 = x - self.delta[action[x][y]][0]
            y2 = y - self.delta[action[x][y]][1]
            policy[x2][y2] = self.delta_name[action[x][y]]
            direction[x2][y2] = 'move ' + self.delta_directions[action[x][y]]
            moves.append('move ' + self.delta_directions[action[x][y]])
            x = x2
            y = y2

        moves.reverse()
        # print('after reverse', moves)
        if len(moves) > 0:
            moves.pop()
        # print('after pop', moves)
        # print('task:', task)
        if task == 'pick up':
            moves.append('lift ' + item)
            #remove the * at the cell where robot was during previous dropping off position, or the 1st starting point
            # print('self.locate("*")',self.locate("*"))
            self.grid[self.locate("*")[0]][self.locate("*")[1]] = '.'
            # update to traversable cell after picking up box
            self.grid[goal[0]][goal[1]] = '.'
            # update drop point with @ because if it was where the robot initially was
            if self.locate("@") == []:
                self.grid[init[0]][init[1]] = '@'

            # update current robot position with *
            self.grid[goal[0]-self.delta[action[goal[0]][goal[1]]][0]][goal[1]-self.delta[action[goal[0]][goal[1]]][1]] = '*'
        elif task == 'drop':
            # if there is a drop zone
            if self.locate("@") != []:
                moves.append('down ' + self.delta_directions[action[goal[0]][goal[1]]])
                # update the cell where robot was in when picking up the box to .
                self.grid[init[0]][init[1]] = '.'
                # update current robot position with *
                self.grid[goal[0]-self.delta[action[goal[0]][goal[1]]][0]][goal[1]-self.delta[action[goal[0]][goal[1]]][1]] = '*'

            else:
                # check for traversable cells
                # found a spot to move over
                found_a_spot = False
                i = 0
                while found_a_spot is False:
                    # if x2 >= 0 and x2 < len(self.grid) and y2 >= 0 and y2 < len(self.grid[0]):
                    m = init[0]+self.delta[i][0]
                    n = init[1]+self.delta[i][1]
                    # self.grid[init[0] + self.delta[i][0]][init[1] + self.delta[i][1]] in ('.'):
                    if m >=0 and n >= 0 and m < len(self.grid) and n < len(self.grid[0]):
                        if self.grid[m][n] == '.':
                            moves.append('move ' + self.delta_directions[i])
                            moves.append('down ' + self.delta_directions_reverse[i])
                            # update the cell where robot was in when picking up the box to @
                            self.grid[init[0]][init[1]] = '@'
                            # update current robot position with *
                            self.grid[m][n] = '*'
                            found_a_spot = True
                    i += 1
        # print('final moves', moves)

        return moves, policy

    def plan_delivery(self, debug=False):
        """
        plan_delivery() is required and will be called by the autograder directly.  
        You may not change the function signature for it.
        Add logic here to find the moves.  You may use the starter code provided above
        in any way you choose, but please condition any printouts on the debug flag
        """

        # Find the moves - you may add arguments and change this logic but please leave
        # the debug flag in place and condition all printouts on it.

        # You may wish to break the task into one-way paths, like this:
        #
        #    moves_to_1   = self._search( ..., debug=debug )
        #    moves_from_1 = self._search( ..., debug=debug )
        #    moves_to_2   = self._search( ..., debug=debug )
        #    moves_from_2 = self._search( ..., debug=debug )
        #    moves        = moves_to_1 + moves_from_1 + moves_to_2 + moves_from_2
        #
        # If you use _search(), you may need to modify it to take some
        # additional arguments for starting location, goal location, and
        # whether to pick up or deliver a box.
        self.grid = copy.deepcopy(self.warehouse_state)


        # locate all boxes:
        boxes_pos = []
        for i in range(len(self.todo)):
            boxes_pos.append(self.locate(self.todo[i]))
            # print('todo[i]',self.todo[i])
            # print(boxes_pos[i])

        moves = []
        policy = []
        for i in range(len(self.todo)):
            result = self._search(task='pick up', item=self.todo[i], debug=debug)
            move1, policy = result
            moves += move1
            # print updated grid after each movement section
            # print('----------After pick up-------------')
            # for j in range(len(self.grid)):
            #     print(self.grid[j])

            # print('----------After drop off-----------------------')
            result = self._search(task='drop', item=self.todo[i],  debug=debug)
            move2, policy = result
            moves += move2
            # for j in range(len(self.grid)):
            #     print(self.grid[j])

        for i in range(len(policy)):
            print(policy[i])

        print(moves)

        if debug:
            for i in range(len(moves)):
                print(moves[i])

        return moves



#########################CREATE A NEW SEARCH FUNCTION FOR PART B #####################################

class DeliveryPlanner_PartB:
    """
    Required methods in this class are:

        plan_delivery(self, debug = False) which is stubbed out below.
        You may not change the method signature as it will be called directly
        by the autograder but you may modify the internals as needed.

        __init__: required to initialize the class.  Starter code is
        provided that initializes class variables based on the definitions in
        testing_suite_partB.py.  You may choose to use this starter code
        or modify and replace it based on your own solution

    The following methods are starter code you may use for part B.
    However, they are not required and can be replaced with your
    own methods.

        _set_initial_state_from(self, warehouse): creates structures based on
            the warehouse and todo definitions and initializes the robot
            location in the warehouse

        _find_policy(self, goal, pickup_box=True, debug=False): Where the bulk 
            of the dynamic programming (DP) search algorithm could reside.  
            It should find an optimal path from the robot location to a goal.
            Hint:  you may want to structure this based
            on whether looking for a box or delivering a box.

    """

    # Definitions taken from testing_suite_partA.py
    ORTHOGONAL_MOVE_COST = 2
    DIAGONAL_MOVE_COST = 3
    BOX_LIFT_COST = 4
    BOX_DOWN_COST = 2
    ILLEGAL_MOVE_PENALTY = 100

    def __init__(self, warehouse, warehouse_cost, todo):

        self.todo = todo
        self.boxes_delivered = []
        self.total_cost = 0
        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost
        self.grid = []
        self.delta = [[-1, 0],  # go up
                      [0, -1],  # go left
                      [1, 0],  # go down
                      [0, 1],  # go right
                      [-1, -1],  # up left (diag)
                      [-1, 1],  # up right (diag)
                      [1, 1],  # dn right (diag)
                      [1, -1]]  # dn left (diag)

        self.delta_directions = ["n", "w", "s", "e", "nw", "ne", "se", "sw"]

        # Use this for a visual debug
        self.delta_name = ['^', '<', 'v', '>', '\\', '/', '[', ']']
        # You may choose to use arrows instead
        # self.delta_name = ['🡑', '🡐', '🡓', '🡒',  '🡔', '🡕', '🡖', '🡗']

        # Costs for each move
        self.delta_cost = [self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST,
                           self.DIAGONAL_MOVE_COST]

    # state parsing and initialization function from testing_suite_partA.py
    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse(list(list)): the warehouse map.
        """
        rows = len(warehouse)
        cols = len(warehouse[0])

        self.warehouse_state = [[None for j in range(cols)] for i in range(rows)]
        self.dropzone = None
        self.boxes = dict()

        for i in range(rows):
            for j in range(cols):
                this_square = warehouse[i][j]

                if this_square == '.':
                    self.warehouse_state[i][j] = '.'

                elif this_square == '#':
                    self.warehouse_state[i][j] = '#'

                elif this_square == '@':
                    self.warehouse_state[i][j] = '*'
                    self.dropzone = (i, j)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)

    def locate(self, target):
        """
        :param target:the character representing the item you want to locate on the grid. Example: 1 in  this case
        :return: a coordinate of that target [i,j]
        """
        nlist = [[i, el.index(target)] for i, el in enumerate(self.grid) if target in el]
        if nlist != []:
            res = nlist[0]
        else:
            res = []
        return res

    def _find_policy(self, goal, pickup_box=True, debug=False):
        """
        This method should be based on lesson modules for Dynamic Programming,
        see Search, Section 15-19 and Problem Set 4, Question 5.  The bulk of
        the logic for finding the policy should reside here should you choose to
        use this starter code.  Please condition any printout on the debug flag
        provided in the argument. You may change this function signature
        (i.e. add arguments) as necessary, except for the debug argument which
        must remain with a default of False
        """

        ##############################################################################
        # insert code in this method if using the starter code we've provided
        ##############################################################################

        # self.warehouse_cost
        # print("in search program now")
        # if pickup_box == True:
        #     goal = self.locate("1")
        # else:
        #     goal = self.locate("*")
        # print(goal)

        value = [[999 for row in range(len(self.grid[0]))] for col in range(len(self.grid))]
        policy = [['-1' for row in range(len(self.grid[0]))] for col in range(len(self.grid))]
        change = True

        while change:
            change = False
            for x in range(len(self.grid)):
                for y in range(len(self.grid[0])):
                    if goal[0] == x and goal[1] == y:
                        if value[x][y] > 0:
                            value[x][y] = 0
                            if pickup_box:
                                policy[x][y] = 'B' #Set box location to B on policy map
                                change = True
                            else: #move out of drop zone before dropping box
                                step_aside_cost = 999
                                for d in range(len(self.delta)):
                                    step_aside_x = x + self.delta[d][0]
                                    step_aside_y = y + self.delta[d][1]

                                    if step_aside_x >= 0 and step_aside_x < len(self.grid) and step_aside_y >= 0 and step_aside_y < len(self.grid[0]) and self.grid[step_aside_x][step_aside_y] == '.':
                                        if self.warehouse_cost[step_aside_x][step_aside_y] < step_aside_cost:
                                            policy[x][y] = 'move ' + self.delta_directions[d]
                                            step_aside_cost = self.warehouse_cost[step_aside_x][step_aside_y] + self.delta_cost[a]
                                        change = True

                    elif self.grid[x][y] != '#':
                        for a in range(len(self.delta)):
                            x2 = x + self.delta[a][0]
                            y2 = y + self.delta[a][1]
                            if x2 >= 0 and x2 < len(self.grid) and y2 >= 0 and y2 < len(self.grid[0]) and self.grid[x2][y2] != '#':
                                v2 = value[x2][y2] + self.warehouse_cost[x2][y2] + self.delta_cost[a]

                                # print("v2", v2)
                                if v2 < value[x][y]:
                                    change = True
                                    value[x][y] = v2
                                    if x2 == goal[0] and y2 == goal[1]:
                                        if pickup_box:
                                            policy[x][y] = 'lift 1'
                                        else:
                                            policy[x][y] = 'down ' + self.delta_directions[a]
                                    else:
                                        policy[x][y] = 'move ' + self.delta_directions[a]

        #     for i in range(len(value)):
        #         print(value[i])
        # print('pickup_box=', pickup_box)
        # for i in range(len(policy)):
        #     print(policy[i])

        # You will need to fill in the algorithm here to find the policy
        # The following are what your algorithm should return for test case 1
        # if pickup_box:
        #     # To box policy
        #     policy = [['B', 'lift 1', 'move w'],
        #               ['lift 1', '-1', 'move nw'],
        #               ['move n', 'move nw', 'move n']]
        #
        # else:
        #     # Deliver policy
        #     policy = [['move e', 'move se', 'move s'],
        #               ['move ne', '-1', 'down s'],
        #               ['move e', 'down e', 'move n']]

        return policy

    def plan_delivery(self, debug=False):
        """
        plan_delivery() is required and will be called by the autograder directly.  
        You may not change the function signature for it.
        Add logic here to find the policies:  First to the box from any grid position
        then to the dropzone, again from any grid position.  You may use the starter
        code provided above in any way you choose, but please condition any printouts
        on the debug flag
        """
        ###########################################################################
        # Following is an example of how one could structure the solution using
        # the starter code we've provided.
        ###########################################################################

        # Start by finding a policy to direct the robot to the box from any grid position
        # The last command(s) in this policy will be 'lift 1' (i.e. lift box 1)
        # goal = self.boxes['1']
        # print('warehouse state')
        # for i in range(len(self.warehouse_state)):
        #     print(self.warehouse_state[i])
        # print('----------------------------------')

        self.grid = copy.deepcopy(self.warehouse_state)
        # print("self.grid")
        # for i in range(len(self.grid)):
        #     print(self.grid[i])
        goal = self.locate("1")
        to_box_policy = self._find_policy(goal, pickup_box=True, debug=debug)

        # Now that the robot has the box, transition to the deliver policy.  The
        # last command(s) in this policy will be 'down x' where x = the appropriate
        # direction to set the box into the dropzone
        #     should be locating @, but warehouse state has the robot on the dropzone, showing *
        goal = self.locate("*")
        # print('locate("*")', goal)
        deliver_policy = self._find_policy(goal, pickup_box=False, debug=debug)


        # if debug:
        #     print("\nTo Box Policy:")
        #     for i in range(len(to_box_policy)):
        #         print(to_box_policy[i])
        #
        #     print("\nDeliver Policy:")
        #     for i in range(len(deliver_policy)):
        #         print(deliver_policy[i])

        return (to_box_policy, deliver_policy)


class DeliveryPlanner_PartC:
    """
    Required methods in this class are:

        plan_delivery(self, debug = False) which is stubbed out below.
        You may not change the method signature as it will be called directly
        by the autograder but you may modify the internals as needed.

        __init__: required to initialize the class.  Starter code is
        provided that initializes class variables based on the definitions in
        testing_suite_partC.py.  You may choose to use this starter code
        or modify and replace it based on your own solution

    The following methods are starter code you may use for part C.
    However, they are not required and can be replaced with your
    own methods.

        _set_initial_state_from(self, warehouse): creates structures based on
            the warehouse and todo definitions and initializes the robot
            location in the warehouse

        _find_policy(self, goal, pickup_box=True, debug=False): 
            Where the bulk of your algorithm could reside.
            It should find an optimal policy to a goal.
            Remember that actions are stochastic rather than deterministic.
            Hint:  you may want to structure this based
            on whether looking for a box or delivering a box.

    """

    # Definitions taken from testing_suite_partA.py
    ORTHOGONAL_MOVE_COST = 2
    DIAGONAL_MOVE_COST = 3
    BOX_LIFT_COST = 4
    BOX_DOWN_COST = 2
    ILLEGAL_MOVE_PENALTY = 100

    def __init__(self, warehouse, warehouse_cost, todo, stochastic_probabilities):

        self.todo = todo
        self.boxes_delivered = []
        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost
        self.stochastic_probabilities = stochastic_probabilities

        self.delta = [
            [-1, 0],  # go up
            [-1, -1],  # up left (diag)
            [0, -1],  # go left
            [1, -1],  # dn left (diag)
            [1, 0],  # go down
            [1, 1],  # dn right (diag)
            [0, 1],  # go right
            [-1, 1],  # up right (diag)]
        ]

        self.delta_directions = ["n", "nw", "w", "sw", "s", "se", "e", "ne"]

        # Use this for a visual debug
        self.delta_name = ['🡑', '🡔', '🡐', '🡗', '🡓', '🡖', '🡒', '🡕']

        # Costs for each move
        self.delta_cost = [self.ORTHOGONAL_MOVE_COST, self.DIAGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST, self.DIAGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST, self.DIAGONAL_MOVE_COST,
                           self.ORTHOGONAL_MOVE_COST, self.DIAGONAL_MOVE_COST, ]

    # state parsing and initialization function from testing_suite_partA.py
    def _set_initial_state_from(self, warehouse):
        """Set initial state.

        Args:
            warehouse(list(list)): the warehouse map.
        """
        rows = len(warehouse)
        cols = len(warehouse[0])

        self.warehouse_state = [[None for j in range(cols)] for i in range(rows)]
        self.dropzone = None
        self.boxes = dict()

        for i in range(rows):
            for j in range(cols):
                this_square = warehouse[i][j]

                if this_square == '.':
                    self.warehouse_state[i][j] = '.'

                elif this_square == '#':
                    self.warehouse_state[i][j] = '#'

                elif this_square == '@':
                    self.warehouse_state[i][j] = '*'
                    self.dropzone = (i, j)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)
    def locate(self, target):
        """
        :param target:the character representing the item you want to locate on the grid. Example: 1 in  this case
        :return: a coordinate of that target [i,j]
        """
        nlist = [[i, el.index(target)] for i, el in enumerate(self.grid) if target in el]
        if nlist != []:
            res = nlist[0]
        else:
            res = []
        return res

    def _find_policy(self, goal, pickup_box=True, debug=False):
        """
        You are free to use any algorithm necessary to complete this task.
        Some algorithms may be more well suited than others, but deciding on the
        algorithm will allow you to think about the problem and understand what
        tools are (in)adequate to solve it. Please condition any printout on the
        debug flag provided in the argument. You may change this function signature
        (i.e. add arguments) as necessary, except for the debug argument which
        must remain with a default of False
        """

        ##############################################################################
        # insert code in this method if using the starter code we've provided
        ##############################################################################

        p = self.stochastic_probabilities
        prob = [p["sideways"], p["slanted"], p["as_intended"], p["slanted"], p["sideways"]]
        # print("prob", prob)
        # You will need to fill in the algorithm here to find the policy
        # The following are what your algorithm should return for test case 1
        value = [[9999 for row in range(len(self.grid[0]))] for col in range(len(self.grid))]
        policy = [['-1' for row in range(len(self.grid[0]))] for col in range(len(self.grid))]
        change = True

        while change:
            change = False
            for x in range(len(self.grid)):
                for y in range(len(self.grid[0])):
                    if goal[0] == x and goal[1] == y:
                        if value[x][y] > 0:
                            value[x][y] = 0
                            if pickup_box:
                                policy[x][y] = 'B' #Set box location to B on policy map
                                change = True
                            else: #move out of drop zone before dropping box
                                ###step aside using probability
                                # step_aside_cost = 999
                                # for d in range(len(self.delta)):
                                #     step_aside_x = x + self.delta[d][0]
                                #     step_aside_y = y + self.delta[d][1]
                                #
                                #     if step_aside_x >= 0 and step_aside_x < len(self.grid) and step_aside_y >= 0 and step_aside_y < len(self.grid[0]) and self.grid[step_aside_x][step_aside_y] == '.':
                                #         if self.warehouse_cost[step_aside_x][step_aside_y] < step_aside_cost:
                                #             policy[x][y] = 'move ' + self.delta_directions[d]
                                #             step_aside_cost = self.warehouse_cost[step_aside_x][step_aside_y] + self.delta_cost[a]
                                #         change = True
                                ###step aside using probability

                                for a in range(len(self.delta)):
                                    v2_curr = 0
                                    v2_min = 1000
                                    for i in range(-2,3): #all possible outcomes
                                        a2 = (a + i) % len(self.delta)
                                        x2 = x + self.delta[a2][0]
                                        y2 = y + self.delta[a2][1]

                                        if x2 >= 0 and x2 < len(self.grid) and y2 >= 0 and y2 < len(self.grid[0]) and self.grid[x2][y2] != '#':
                                            v2_curr += prob[i + 2] * (value[x2][y2] + self.warehouse_cost[x2][y2] + self.delta_cost[a2])
                                            # print('x:', x, 'y:', y, 'a:', a, 'i:', i, 'a2:', a2, 'x2:', x2, 'y2:', y2,'self.grid[x2][y2]:', self.grid[x2][y2], 'v2:', v2, 'prob[i+2]:', prob[i + 2],'value[x2][y2]:', value[x2][y2], 'value[x][y]:', value[x][y])
                                        else:
                                            v2_curr += prob[i + 2] * (value[x][y] + self.ILLEGAL_MOVE_PENALTY)  # if illegal move, cost plus current cell value
                                            # print('x:', x, 'y:', y, 'a:', a, 'i:', i, 'a2:', a2, 'v2:', v2, 'prob[i+2]:', prob[i + 2],'value[x][y]:', value[x][y],'self.ILLEGAL_MOVE_PENALTY:',self.ILLEGAL_MOVE_PENALTY)
                                        # print("v2", v2)
                                    if v2_curr < v2_min:
                                        v2_min = v2_curr
                                        change = True
                                        # value[x][y] = v2_min
                                        # if x == goal[0] and y2 == goal[1]:
                                        policy[x][y] = 'move ' + self.delta_directions[a]

                    elif self.grid[x][y] != '#':
                        for a in range(len(self.delta)):
                            if x + self.delta[a][0] == goal[0] and y + self.delta[a][1] == goal[1]:
                                if pickup_box:
                                    policy[x][y] = 'lift 1'
                                else:
                                    policy[x][y] = 'down ' + self.delta_directions[a]
                            v2 = 0
                            for i in range(-2, 3): #all possible outcomes
                                a2 = (a + i) % len(self.delta)
                                x2 = x + self.delta[a2][0]
                                y2 = y + self.delta[a2][1]

                                # v2 = 0
                                if x2 >= 0 and x2 < len(self.grid) and y2 >= 0 and y2 < len(self.grid[0]) and self.grid[x2][y2] != '#':
                                    v2 += prob[i+2] * (value[x2][y2] + self.warehouse_cost[x2][y2] + self.delta_cost[a2])
                                    # print('x:', x, 'y:', y, 'a:', a, 'i:', i, 'a2:', a2, 'x2:', x2, 'y2:', y2,'self.grid[x2][y2]:', self.grid[x2][y2], 'v2:', v2, 'prob[i+2]:', prob[i + 2],'value[x2][y2]:', value[x2][y2], 'value[x][y]:', value[x][y])
                                else:
                                    v2 += prob[i+2] * (value[x][y] + self.ILLEGAL_MOVE_PENALTY) #if illegal move, cost plus current cell value
                                    # print('x:', x, 'y:', y, 'a:', a, 'i:', i, 'a2:', a2, 'v2:', v2, 'prob[i+2]:', prob[i + 2],'value[x][y]:', value[x][y],'self.ILLEGAL_MOVE_PENALTY:',self.ILLEGAL_MOVE_PENALTY)
                                # print("v2", v2)
                            if v2 < value[x][y]:

                                change = True
                                value[x][y] = v2
                                # if x == goal[0] and y2 == goal[1]:
                                policy[x][y] = 'move ' + self.delta_directions[a]
                                # print(policy[x][y])
                        # for a in range(len(self.delta)):
                        #     x2 = x + self.delta[a][0]
                        #     y2 = y + self.delta[a][1]
                        #     if x2 >= 0 and x2 < len(self.grid) and y2 >= 0 and y2 < len(self.grid[0]) and self.grid[x2][y2] != '#':
                        #         v2 = value[x2][y2] + self.warehouse_cost[x2][y2] + self.delta_cost[a]
                        #
                        #         # print("v2", v2)
                        #         if v2 < value[x][y]:
                        #             change = True
                        #             value[x][y] = v2
                        #             if x2 == goal[0] and y2 == goal[1]:
                        #                 if pickup_box:
                        #                     policy[x][y] = 'lift 1'
                        #                 else:
                        #                     policy[x][y] = 'down ' + self.delta_directions[a]
                        #             else:
                        #                 policy[x][y] = 'move ' + self.delta_directions[a]


                # print('-------------------------------')
                # for i in range(len(value)):
                #     print(value[i])
                # print('-------------------------------')
                # for i in range(len(policy)):
                #     print(policy[i])
        return policy, value

    def plan_delivery(self, debug=False):
        """
        plan_delivery() is required and will be called by the autograder directly.
        You may not change the function signature for it.
        Add logic here to find the policies:  First to the box from any grid position
        then to the dropzone, again from any grid position.  You may use the starter
        code provided above in any way you choose, but please condition any printouts
        on the debug flag
        """
        ###########################################################################
        # Following is an example of how one could structure the solution using
        # the starter code we've provided.
        ###########################################################################
        # Get copy of the warehouse state
        self.grid = copy.deepcopy(self.warehouse_state)
        # Start by finding a policy to direct the robot to the box from any grid position
        # The last command(s) in this policy will be 'lift 1' (i.e. lift box 1)
        # goal = self.boxes['1']
        # to_box_policy = self._find_policy(goal, pickup_box=True, debug=debug)
        #
        # # Now that the robot has the box, transition to the deliver policy.  The
        # # last command(s) in this policy will be 'down x' where x = the appropriate
        # # direction to set the box into the dropzone
        # goal = self.dropzone
        # to_zone_policy = self._find_policy(goal, pickup_box=False, debug=debug)
        goal = self.locate("1")
        # print('locate("1")', goal)
        to_box_policy, to_box_values = self._find_policy(goal, pickup_box=True, debug=debug)

        # Now that the robot has the box, transition to the deliver policy.  The
        # last command(s) in this policy will be 'down x' where x = the appropriate
        # direction to set the box into the dropzone
        #     should be locating @, but warehouse state has the robot on the dropzone, showing *
        goal = self.locate("*")
        # print('locate("*")', goal)
        to_zone_policy, to_zone_values = self._find_policy(goal, pickup_box=False, debug=debug)

        if debug:
            print("\nTo Box Policy:")
            for i in range(len(to_box_policy)):
                print(to_box_policy[i])

            print("\nDeliver Policy:")
            for i in range(len(to_zone_policy)):
                print(to_zone_policy[i])

        print("\nTo Box Policy:")
        for i in range(len(to_box_policy)):
            print(to_box_policy[i])
        print("\nTo Box Value:")
        for i in range(len(to_box_values)):
            print(to_box_values[i])
        print("\nDeliver Policy:")
        for i in range(len(to_zone_policy)):
            print(to_zone_policy[i])
        print("\nTo Zone Value:")
        for i in range(len(to_zone_values)):
            print(to_zone_values[i])

        # For debugging purposes you may wish to return values associated with each policy.
        # Replace the default values of None with your grid of values below and turn on the
        # VERBOSE_FLAG in the testing suite.
        # to_box_values = None
        # to_zone_values = None
        return (to_box_policy, to_zone_policy, to_box_values, to_zone_values)

 

if __name__ == "__main__":
    """ 
    You may execute this file to develop and test the search algorithm prior to running 
    the delivery planner in the testing suite.  Copy any test cases from the
    testing suite or make up your own.
    Run command:  python warehouse.py
    """

    # Test code in here will not be called by the autograder

    # Testing for Part A
    # testcase 1
    print('\nTesting for part A:')
    warehouse = ['1#2',
                 '.#.',
                 '..@']

    todo = ['1', '2']

    partA = DeliveryPlanner_PartA(warehouse, todo)
    partA.plan_delivery(debug=True)

    # Testing for Part B
    # testcase 1
    print('\nTesting for part B:')
    warehouse = ['1..',
                 '.#.',
                 '..@']

    warehouse_cost = [[3, 5, 2],
                      [10, math.inf, 2],
                      [2, 10, 2]]

    todo = ['1']

    partB = DeliveryPlanner_PartB(warehouse, warehouse_cost, todo)
    partB.plan_delivery(debug=True)

    # Testing for Part C
    # testcase 1
    print('\nTesting for part C:')
    warehouse = ['1..',
                 '.#.',
                 '..@']

    warehouse_cost = [[13, 5, 6],
                      [10, math.inf, 2],
                      [2, 11, 2]]

    todo = ['1']

    stochastic_probabilities = {
        'as_intended': .70,
        'slanted': .1,
        'sideways': .05,
    }

    partC = DeliveryPlanner_PartC(warehouse, warehouse_cost, todo, stochastic_probabilities)
    partC.plan_delivery(debug=True)
