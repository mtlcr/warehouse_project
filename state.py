
import copy
import string
import sys


DIRECTIONS = 'n,nw,w,sw,s,se,e,ne'.split(',')
DIRECTION_INDICES = {direction: index for index, direction in enumerate(DIRECTIONS)}
DELTA_DIRECTIONS = [
    (-1, 0),
    (-1, -1),
    (0, -1),
    (1, -1),
    (1, 0),
    (1, 1),
    (0, 1),
    (-1, 1),
]

class State:
    """Current State.

    Args:
        warehouse(list(list)): the warehouse map.
        warehouse_cost(list(list)): integer costs for each warehouse position
        robot_initial_position(i,j): robot's initial position

    Attributes:
        boxes_delivered(list): the boxes successfully delivered to dropzone.
        total_cost(int): the total cost of all moves executed.
        warehouse_state(list(list): the current warehouse state.
        dropzone(tuple(int, int)): the location of the dropzone.
        boxes(list): the location of the boxes.
        robot_position(tuple): the current location of the robot.
        box_held(str): ID of current box held.
    """
    ORTHOGONAL_MOVE_COST = 2
    DIAGONAL_MOVE_COST = 3
    BOX_LIFT_COST = 4
    BOX_DOWN_COST = 2
    ILLEGAL_MOVE_PENALTY = 100

    def __init__(self, warehouse, warehouse_cost=None, robot_init=None):
        self.boxes_delivered = []
        self.total_cost = 0
        self.robot_position = copy.copy(robot_init) if robot_init else None

        self._set_initial_state_from(warehouse)
        self.warehouse_cost = warehouse_cost if warehouse_cost else [[0 for _ in row] for row in warehouse]

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
                    self.warehouse_state[i][j] = '@'
                    self.dropzone = (i, j)

                else:  # a box
                    box_id = this_square
                    self.warehouse_state[i][j] = box_id
                    self.boxes[box_id] = (i, j)

        if not self.robot_position:
            self.robot_position = self.dropzone

        self.warehouse_state[self.robot_position[0]][self.robot_position[1]] = '*'

        self.box_held = None

    def update_according_to(self, action):
        """Update state according to action.

        Args:
            action(str): action to execute.

        Raises:
            Exception: if improperly formatted action.
        """
        # what type of move is it?
        action = action.split()
        action_type = action[0]

        if action_type == 'move':
            direction = action[1]
            self._attempt_move(direction)

        elif action_type == 'lift':
            box = action[1]
            self._attempt_lift(box)

        elif action_type == 'down':
            direction = action[1]
            self._attempt_down(direction)

        else:
            # improper move format: kill test
            raise Exception('improperly formatted action: {}'.format(''.join(action)))

    def _attempt_move(self, direction):
        """Attempt move action if valid.

        The robot may not move outside the warehouse.
        The warehouse does not "wrap" around.
        Two spaces are considered adjacent if they share an edge or a corner.

        The robot may move horizontally or vertically at a cost of 2 per move.
        The robot may move diagonally at a cost of 3 per move.
        Illegal move (100 cost):
            attempting to move to a nonadjacent, nonexistent, or occupied space

        Args:
            direction: direction in which to move to adjacent square
                ("n","ne","e","se","s","sw","w","nw")


        Raises:
            ValueError: if improperly formatted move destination.
            IndexError: if move is outside of warehouse.
        """
        try:

            destination = DELTA_DIRECTIONS[DIRECTION_INDICES[direction]][0] + self.robot_position[0], \
                          DELTA_DIRECTIONS[DIRECTION_INDICES[direction]][1] + self.robot_position[1]

            destination_is_adjacent = self._are_adjacent(self.robot_position, destination)
            destination_is_traversable = self._is_traversable(destination)
            destination_is_within_warehouse = self._is_within_warehouse(destination)

            is_legal_move = (destination_is_adjacent and
                             destination_is_traversable and
                             destination_is_within_warehouse)

            if is_legal_move:
                self._move_robot_to(destination)
            else:
                action_cost = self.DIAGONAL_MOVE_COST if len(direction)==2 else self.ORTHOGONAL_MOVE_COST
                self._increase_total_cost_by(self.ILLEGAL_MOVE_PENALTY + action_cost)

        except ValueError:
            raise Exception(
                "move direction must be 'n','ne','e','se','s','sw','w','nw' your move is: {}".format(direction))
        except IndexError:  # (row, col) not in warehouse
            # self._increase_total_cost_by(self.ILLEGAL_MOVE_PENALTY)
            action_cost = self.DIAGONAL_MOVE_COST if len(direction) == 2 else self.ORTHOGONAL_MOVE_COST
            self._increase_total_cost_by(self.ILLEGAL_MOVE_PENALTY + action_cost)


    def _attempt_lift(self, box_id):
        """Attempt lift action if valid.

        The robot may pick up a box that is in an adjacent square.
        The cost to pick up a box is 4, regardless of the direction the box is relative to the robot.
        While holding a box, the robot may not pick up another box.
        Illegal moves (100 cost):
            attempting to pick up a nonadjacent or nonexistent box
            attempting to pick up a box while holding one already

        Args:
            box_id(str): the id of the box to lift.

        Raises:
            KeyError: if invalid box id.
        """
        try:
            box_position = self.boxes[box_id]

            box_is_adjacent = self._are_adjacent(self.robot_position, box_position)
            robot_has_box = self._robot_has_box()

            is_legal_lift = box_is_adjacent and (not robot_has_box)
            if is_legal_lift:
                self._lift_box(box_id)
            else:
                self._increase_total_cost_by(self.ILLEGAL_MOVE_PENALTY + self.BOX_LIFT_COST)

        except KeyError:
            self._increase_total_cost_by(self.ILLEGAL_MOVE_PENALTY + self.BOX_LIFT_COST)

    def _attempt_down(self, direction):
        """Attempt down action if valid.

        The robot may put a box down on an adjacent empty space ('.') or the dropzone ('@') at a cost
            of 2 (regardless of the direction in which the robot puts down the box).
        Illegal moves (100 cost):
            attempting to put down a box on a nonadjacent, nonexistent, or occupied space
            attempting to put down a box while not holding one

        Args:
            direction: direction to adjacent square in which to set box down 
                  ("n","ne","e","se","s","sw","w","nw")

        Raises:
            ValueError: if improperly formatted down destination.
            IndexError: if down location is outside of warehouse.
        """
        try:
            destination = DELTA_DIRECTIONS[DIRECTION_INDICES[direction]][0] + self.robot_position[0], \
                          DELTA_DIRECTIONS[DIRECTION_INDICES[direction]][1] + self.robot_position[1]

            destination_is_adjacent = self._are_adjacent(self.robot_position, destination)
            destination_is_traversable = self._is_traversable(destination)
            destination_is_within_warehouse = self._is_within_warehouse(destination)
            robot_has_box = self._robot_has_box()

            is_legal_down = destination_is_adjacent and destination_is_traversable and robot_has_box and destination_is_within_warehouse
            if is_legal_down:
                self._down_box(destination)
            else:
                self._increase_total_cost_by(self.ILLEGAL_MOVE_PENALTY + self.BOX_DOWN_COST)

        except ValueError:
            raise Exception('improperly formatted down destination: {}'.format(direction))
        except IndexError:  # (row, col) not in warehouse
            self._increase_total_cost_by(self.ILLEGAL_MOVE_PENALTY + self.BOX_DOWN_COST)

    def _increase_total_cost_by(self, amount):
        """Increase total move cost.

        Args:
            amount(int): amount to increase cost by.
        """
        self.total_cost += amount

    def _is_within_warehouse(self, coordinates):
        """Check if coordinates are within warehouse.

        Args:
            coordinates(tuple(int, int)): coordinates to test.

        Returns:
            True if within warehouse.
        """
        i, j = coordinates
        rows = len(self.warehouse_state)
        cols = len(self.warehouse_state[0])

        return (0 <= i < rows) and (0 <= j < cols)

    def _are_adjacent(self, coordinates1, coordinates2):
        """Verify if coordinates are adjacent.

        Args:
            coordinates1(tuple(int, int)): first coordinate.
            coordinates2(tuple(int, int)): second coordinate.

        Returns:
            True if adjacent in all directions.
        """
        return (self._are_horizontally_adjacent(coordinates1, coordinates2) or
                self._are_vertically_adjacent(coordinates1, coordinates2) or
                self._are_diagonally_adjacent(coordinates1, coordinates2)
                )

    @staticmethod
    def _are_horizontally_adjacent(coordinates1, coordinates2):
        """Verify if coordinates are horizontally adjacent.

        Args:
            coordinates1(tuple(int, int)): first coordinate.
            coordinates2(tuple(int, int)): second coordinate.

        Returns:
            True if horizontally adjacent.
        """
        row1, col1 = coordinates1
        row2, col2 = coordinates2

        return (row1 == row2) and (abs(col1 - col2) == 1)

    @staticmethod
    def _are_vertically_adjacent(coordinates1, coordinates2):
        """Verify if coordinates are vertically adjacent.

        Args:
            coordinates1(tuple(int, int)): first coordinate.
            coordinates2(tuple(int, int)): second coordinate.

        Returns:
            True if vertically adjacent.
        """
        row1, col1 = coordinates1
        row2, col2 = coordinates2

        return (abs(row1 - row2) == 1) and (col1 == col2)

    @staticmethod
    def _are_diagonally_adjacent(coordinates1, coordinates2):
        """Verify if coordinates are diagonally adjacent.

        Args:
            coordinates1(tuple(int, int)): first coordinate.
            coordinates2(tuple(int, int)): second coordinate.

        Returns:
            True if diagonally adjacent.
        """
        row1, col1 = coordinates1
        row2, col2 = coordinates2

        return (abs(row1 - row2) == 1) and (abs(col1 - col2) == 1)

    def _is_traversable(self, coordinates):
        """Verify if space is traversable.

        Args:
            coordinates(tuple(int, int)): coordinate to check.

        Return:
            True if traversable.
        """
        is_wall = self._is_wall(coordinates)
        has_box = self._space_contains_box(coordinates)

        return (not is_wall) and (not has_box)

    def _is_wall(self, coordinates):
        """Verify if space is wall.

        Args:
            coordinates(tuple(int, int)): coordinate to check.

        Return:
            True if wall.
        """
        i, j = coordinates

        return self.warehouse_state[i][j] == '#'

    def _space_contains_box(self, coordinates):
        """Verify if space contains box.

        Args:
            coordinates(tuple(int, int)): coordinate to check.

        Return:
            True if space contains box.
        """
        i, j = coordinates

        return self.warehouse_state[i][j] in (string.ascii_letters + string.digits)

    def _robot_has_box(self):
        """Verify if robot has box.

        Returns:
            True if box is being held.
        """
        return self.box_held is not None

    def _move_robot_to(self, destination):
        """Execute move.

        Args:
            destination(tuple(int, int)): location to set box down at.
        """
        old_position = self.robot_position
        self.robot_position = destination

        i1, j1 = old_position
        if self.dropzone == old_position:
            self.warehouse_state[i1][j1] = '@'
        else:
            self.warehouse_state[i1][j1] = '.'

        i2, j2 = destination
        self.warehouse_state[i2][j2] = '*'

        if self._are_diagonally_adjacent(old_position, destination):
            self._increase_total_cost_by(self.DIAGONAL_MOVE_COST)
        else:
            self._increase_total_cost_by(self.ORTHOGONAL_MOVE_COST)

        # Account for the cost of each square
        self._increase_total_cost_by(self.warehouse_cost[i2][j2])

    def _lift_box(self, box_id):
        """Execute lift box.

        Args:
            box_id(str): the id of the box to lift.
        """
        i, j = self.boxes[box_id]
        self.warehouse_state[i][j] = '.'

        self.boxes.pop(box_id)

        self.box_held = box_id

        self._increase_total_cost_by(self.BOX_LIFT_COST + self.warehouse_cost[i][j])

    def _down_box(self, destination):
        """Execute box down.

        Args:
            destination(tuple(int, int)): location to set box down at.
        """
        # - If a box is placed on the '@' space, it is considered delivered and is removed from the ware-
        #   house.
        i, j = destination

        if self.warehouse_state[i][j] == '.':
            self.warehouse_state[i][j] = self.box_held
            self.boxes[self.box_held] = (i, j)
        else:
            self._deliver_box(self.box_held)

        self.box_held = None
        self._increase_total_cost_by(self.BOX_DOWN_COST + self.warehouse_cost[i][j])

    def _deliver_box(self, box_id):
        """Mark box delivered.

        Args:
            box_id(str): id of box to mark delivered.
        """
        self.boxes_delivered.append(box_id)

    def get_boxes_delivered(self):
        """Get list of boxes delivered.

        Returns:
            List of boxes delivered.
        """
        return self.boxes_delivered

    def get_total_cost(self):
        """Get current total cost.

        Returns:
            Total cost of all executed moves.
        """
        return self.total_cost

    def print_to_console(self, fout=None):
        """Print current state to console.
        """
        my_fout = fout or sys.stdout
        my_fout.write("\n")
        for row in self.warehouse_state:
            my_fout.write(''.join(str(row)) + '\n')
        my_fout.write('total cost: %.02f\n' % self.total_cost)
        my_fout.write('box held: %s\n' % str(self.box_held))
        my_fout.write('delivered: %s\n' % str(self.boxes_delivered))
        my_fout.write('\n')
