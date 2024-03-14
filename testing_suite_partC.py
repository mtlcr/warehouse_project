
import unittest
import multiprocessing as mproc
import traceback
import sys
import copy
import io
import math
import random
from state import State


try:
    from warehouse import DeliveryPlanner_PartC, who_am_i
    studentExc = None
except Exception as e:
    studentExc = traceback.format_exc()

########################################################################
# For debugging this flag can be set to True to print state 
# which could result in a timeout
########################################################################
VERBOSE_FLAG = False

########################################################################
# For visualization this flag can be set to True to display a GUI
# which could result in a timeout, but useful for debugging
# Note that enabling this will also enable DEBUGGING_SINGLE_PROCESS
########################################################################
VISUALIZE_FLAG = True

########################################################################
# For debugging set the time limit to a big number (like 600 or more)
########################################################################
TIME_LIMIT = 5  # seconds

########################################################################
# If your debugger does not handle multiprocess debugging very easily
# then when debugging set the following flag true.
########################################################################
DEBUGGING_SINGLE_PROCESS = False

# Necessary for GUI visualization, don't modify these lines
if VISUALIZE_FLAG:
    from visualizer import GUI
DEBUGGING_SINGLE_PROCESS = True if VISUALIZE_FLAG else DEBUGGING_SINGLE_PROCESS

########################################################################
# If your system is not set up to display unicode correctly then you
# can switch this flag to False which will output an alternative
# set of characters in place of the arrows.
########################################################################
DISPLAY_UNICODE_ARROWS = True

DIRECTIONS = 'n,nw,w,sw,s,se,e,ne'.split(',')
ALT_CHARS = '^,`,<,],v,[,>,%'
ARROWS = ('↑,↖,←,↙,↓,↘,→,↗' if DISPLAY_UNICODE_ARROWS else ALT_CHARS).split(',')
DIRECTION_ARROW_DICT = {d: a for d, a in zip(DIRECTIONS, ARROWS)}
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
LEGEND = {
    '-1': '▪' if DISPLAY_UNICODE_ARROWS else 'w',  # wall
    'lift': '+',  # lift action
    'down': '-',  # down action
    'B': '▫' if DISPLAY_UNICODE_ARROWS else 'B',  # box
}


def truncate_output(s, max_len=2000):
    if len(s) > max_len:
        return s[:max_len - 70] + "\n***************** OUTPUT TRUNCATED DUE TO EXCESSIVE LENGTH!**************\n"
    else:
        return s


def get_stochastic_probabilities(prob_as_intended):
    prob_not_as_intended = 100 - prob_as_intended
    prob_slanted = (prob_not_as_intended / 3) / 100
    prob_sideways = (prob_not_as_intended / 6) / 100
    prob_as_intended /= 100
    outcomes = {
        'as_intended': prob_as_intended,
        'slanted': prob_slanted,
        'sideways': prob_sideways,
    }
    assert math.isclose(prob_as_intended + 2 * prob_slanted + 2 * prob_sideways, 1), "Warning: probabilities don't sum to 1"
    return outcomes


def symbol_lookup(action):
    '''Translate the action from words to a single character symbol '''
    action_lst = str(action).split()
    symbol = LEGEND.get(action_lst[0], None)
    if symbol is None:
        symbol = DIRECTION_ARROW_DICT.get(action_lst[1])
    return symbol


def display_policy(policy, values=None, description='Policy'):
    '''
    Print the policy (and associated values) to the console in a condensed format using symbols instead of words.
    Also includes row and col indexes for ease of use.
    '''
    num_rows = len(policy)
    num_cols = len(policy[0])
    col_width = 1
    wall = symbol_lookup('-1')
    box = symbol_lookup('B')

    symbol_policy = []
    for row_index in range(num_rows):
        symbols_row = ''.join([symbol_lookup(action) for action in policy[row_index]])
        symbol_policy.append(symbols_row)

    formatted_policy = symbol_policy
    if values is not None:
        col_width += len(str(max((int(c) if c != float('inf') else 999)
                                 for ir, r in enumerate(values)
                                 for ic, c in enumerate(r)
                                 if symbol_policy[ir][ic] != wall))) + 1

        formatted_policy = []
        int_values = [[(int(v) if v != float('inf') else 'inf') for v in row] for row in values]
        for v_row, s_row in zip(int_values, symbol_policy):
            values_and_symbols_row = ''.join(
                [f' {(str(v) if s not in (box,wall) else " ").rjust(col_width - 2)}{s}'
                 for v, s in zip(v_row, s_row)]) + ' '
            formatted_policy.append(values_and_symbols_row)

    indent_spaces = 5
    indent = " " * indent_spaces
    print(f"_____ {description} _____")
    print(f" (row x col) ==> ({num_rows} x {num_cols})\n")

    col_indexes_tens = ''.join([(str(i // 10) if i > 9 else ' ').rjust(col_width) for i in range(num_cols)])
    col_indexes_ones = ''.join([(str(i)[-1]).rjust(col_width) for i in range(num_cols)])
    col_index_labels = f'{indent}{col_indexes_tens}\n' \
                       f'{indent}{col_indexes_ones}'
    border_top_bottom = f'{indent}{"~" * (num_cols * col_width + (1 if values else 0))}'

    formatted_policy_with_row_index_labels = []
    for i, r in enumerate(formatted_policy):
        tens = str(i // 10) if i > 9 else ' '
        ones = str(i)[-1]
        formatted_policy_with_row_index_labels.append(f'{tens.rjust(indent_spaces - 3)}{ones} |{r}| {tens}{ones}')

    print(col_index_labels)
    print(border_top_bottom)
    print('\n'.join(formatted_policy_with_row_index_labels))
    print(border_top_bottom)
    print(col_index_labels, '\n')


class Submission:
    """Student Submission.
    Attributes:
        submission_score(Queue): Student score of last executed plan.
        submission_error(Queue): Error messages generated during last executed plan.
    """

    def __init__(self, fout=None):

        if DEBUGGING_SINGLE_PROCESS:
            import queue
            self.submission_score = queue.Queue(1)
            self.student_policy = queue.Queue(1)
            self.submission_error = queue.Queue(1)
            self.logmsgs = queue.Queue(1)
        else:
            self.submission_score = mproc.Manager().Queue(1)
            self.student_policy = mproc.Manager().Queue(1)
            self.submission_error = mproc.Manager().Queue(1)
            self.logmsgs = mproc.Manager().Queue(1)

        self.fout = io.StringIO()

    def log(self, s):
        self.fout.write(s + '\n')

    def _reset(self):
        """Reset submission results.
        """
        while not self.submission_score.empty():
            self.submission_score.get()

        while not self.submission_error.empty():
            self.submission_error.get()

        while not self.logmsgs.empty():
            self.logmsgs.get()

    def _get_actions_using_policy(self,
                                 state,
                                 policy,
                                 stochastic_probabilities,
                                 seed):
        """ Extract the set of actions using the policy generated by
            the student planner
        Args:
            state(State): warehouse representation to keep track of robot location and action validation
            policy(list(list)): the warehouse policy
            stochastic_probabilities(dict(string:float)): provides probabilities for 3 outcomes: as_intended, slanted, sideways
            seed(int): used to set random number generator stream for consistency (same used to generate answers)
        """
        MAX_ACTIONS = 400  # max number of actions to retrieve from policy
        i2, j2 = state.robot_position
        actions_performed = []

        random.seed(seed)

        # if there is a policy start moving through it from the initial
        # robot position
        if policy and len(policy[0]) > 0:
            # check that there is a valid policy defined at the initial robot location
            if not (0 <= i2 < len(policy)) or not (j2 < len(policy[0])):
                raise Exception(
                    'Error in _get_actions_from_policy(): no policy defined at initial robot location (i,j)=({},'
                    '{}))'.format(
                        i2, j2))

            stochastic_probabilities_in_index_order = [
                stochastic_probabilities['sideways'],
                stochastic_probabilities['slanted'],
                stochastic_probabilities['as_intended'],
                stochastic_probabilities['slanted'],
                stochastic_probabilities['sideways']
            ]

            # work through the policy for up to MAX_ACTIONS
            while len(actions_performed) < MAX_ACTIONS:
                i, j = i2, j2

                if not isinstance(policy[i][j], str):
                    raise Exception(
                        'Error in _get_actions_from_policy(): command is not of type string at (i,j) = ({},{}), '
                        'action = {}'.format(
                            i, j, policy[i][j]))
                elif len(policy[i][j]) == 0:
                    raise Exception('Error in _get_actions_from_policy(): no command at (i,j)=({},{})'.format(i, j))
                elif '-1' in policy[i][j]:
                    # Note to self, the initial robot location will never be on an actual wall
                    raise Exception(
                        "Error in _get_actions_from_policy(): last action is moving onto grid with '-1' (i,j) = ({},"
                        "{}), action = {}".format(
                            i2, j2, policy[i2][j2]))

                # check for the "final" move which will be a lift or down command
                if 'lift' in policy[i][j] or 'down' in policy[i][j]:
                    actions_performed.append(policy[i][j])
                    break

                # extract the intended action
                intended_action = policy[i][j].split()
                intended_action_index = DIRECTIONS.index(intended_action[1])
                stochasticity_index_delta = random.choices(population=[-2, -1, 0, 1, 2], weights=stochastic_probabilities_in_index_order)[0]
                stochastic_action = DIRECTIONS[(intended_action_index + stochasticity_index_delta) % len(DIRECTIONS)]

                # append it to the list of moves
                actions_performed.append(f'move {stochastic_action}')

                # increment the robot position according to the actual action
                state._attempt_move(stochastic_action)

                i2, j2 = state.robot_position


        else:
            if not policy:
                raise Exception('Error in _get_actions_from_policy(): no policy')
            else:
                raise Exception('Error in _get_actions_from_policy(): policy appears to be empty')

        return actions_performed

    def compare_student_policy(self,
                               test_case,
                               warehouse,
                               warehouse_cost,
                               robot_init,
                               robot_init2,
                               boxes_todo,
                               prob_as_intended,
                               correct_performed_actions,
                               seed):
        """Execute student policy and store results in submission.
        Args:
            test_case(int): number to identify which test case is running (for display purposes)
            warehouse(list(list)): the warehouse map to test against
            warehouse_cost(list(list)): integer costs for each warehouse position
            robot_init (i,j): initial position of robot
            boxes_todo(list): the order of boxes to deliver
            prob_as_intended(float): probability of moving as intended
            correct_performed_actions(string): string of symbols representing the expected actions for a correct policy and particular seed
            seed(int): used to set random number generator stream for consistency (same used to generate answers)
        """
        self._reset()

        state = State(warehouse, warehouse_cost, robot_init=robot_init)
        stochastic_probabilities = get_stochastic_probabilities(prob_as_intended)

        try:
            student_planner = DeliveryPlanner_PartC(copy.deepcopy(warehouse),
                                                    copy.deepcopy(warehouse_cost),
                                                    copy.deepcopy(boxes_todo),
                                                    copy.deepcopy(stochastic_probabilities), )

            to_box_policy, to_zone_policy, to_box_values, to_zone_values = student_planner.plan_delivery(debug=VERBOSE_FLAG)

            if VERBOSE_FLAG:
                border = '||' + '=' * 26 + '||'
                print('\n' + border)
                print(f'  <     Test case # {test_case}    >')
                print(border)
                display_policy(to_box_policy, to_box_values, description='To Box Policy')
                display_policy(to_zone_policy, to_zone_values, description='To Zone Policy')

            score = []
            quit_signal = None
            for desc, policy in zip(['to_box', 'to_zone'], [to_box_policy, to_zone_policy]):
                if desc == 'to_zone':
                    cur_x, cur_y = cur_robot_position = state.robot_position
                    if cur_robot_position != robot_init2:
                        state.warehouse_state[cur_x][cur_y] = '.'
                        init_x, init_y = state.robot_position = robot_init2
                        state.warehouse_state[init_x][init_y] = '*'
                    box_id = '1'
                    if state.box_held is None:
                        i, j = state.boxes[box_id]
                        state.warehouse_state[i][j] = '.'
                        state.boxes.pop(box_id)
                        state.box_held = box_id


                student_performed_actions = self._get_actions_using_policy(copy.deepcopy(state),
                                                                policy,
                                                                stochastic_probabilities,
                                                                seed, )


                if VISUALIZE_FLAG and not quit_signal:
                    gui = GUI('C', test_case, state, len(student_performed_actions))
                    quit_signal = quit_signal if quit_signal is not None else gui.quit_signal
                    if quit_signal:
                        self.log('GUI received quit signal before executing any actions.')
                        break
                    prev_loc = state.robot_position
                    prev_box_locs = copy.deepcopy(state.boxes)

                    for action in student_performed_actions:
                        if VERBOSE_FLAG:
                            state.print_to_console(self.fout)

                        if isinstance(action,str) and len(action) > 0:
                            state.update_according_to(action)

                        if VISUALIZE_FLAG:
                            gui.update(state, action, prev_loc, prev_box_locs)
                            quit_signal = gui.quit_signal
                            if quit_signal:
                                self.log('GUI received quit signal.')
                                break
                            prev_loc = state.robot_position
                            prev_box_locs = copy.deepcopy(state.boxes)

                student_performed_actions = ''.join([symbol_lookup(action) for action in student_performed_actions])

                max_num_actions = max(len(correct_performed_actions[desc]), len(student_performed_actions))
                student_performed_actions = student_performed_actions.ljust(max_num_actions)
                correct_performed_actions[desc] = correct_performed_actions[desc].ljust(max_num_actions)

                actions_correct = [s == e for s, e in zip(student_performed_actions, correct_performed_actions[desc])]
                score.append(.5 if all(actions_correct) else 0)

                if VERBOSE_FLAG:
                    divider = ''  # can switch this to '|' for (possibly) easier readout alignment
                    correct_performed_actions_count = str(len(correct_performed_actions[desc].strip())).rjust(2)
                    student_performed_actions_count = str(len(student_performed_actions.strip())).rjust(2)

                    print(f'| {desc.replace("_", " ").capitalize()}:')
                    print(f'|  Correct actions performed [{correct_performed_actions_count}]: {divider.join(list(correct_performed_actions[desc]))}')
                    print(f'|  Student actions performed [{student_performed_actions_count}]: {divider.join(list(student_performed_actions))}')
                    print(f'|                     Differences: {divider.join([(" " if ac else "^") for ac in actions_correct])}\n')

            self.submission_score.put(score)

        except Exception as err:
            if VERBOSE_FLAG:
                # very detailed stack trace - clutters everything up
                self.submission_error.put(traceback.format_exc())
            else:
                # slightly less cluttered output but the stack trace is much less informative
                self.submission_error.put(err)
            self.submission_score.put((0,0))

        self.logmsgs.put(truncate_output(self.fout.getvalue()))


class PartCTestCase(unittest.TestCase):
    """ Test Part C.
    """

    results = ['', 'PART C TEST CASE RESULTS', 'credit: [to box] + [to zone] = [total]']
    SCORE_TEMPLATE = "\n".join((
        "-----------",
        "Test Case {test_case}",
        "credit: {to_box_score:.1f} + {to_zone_score:.1f} = {total_score:.1f}"
    ))
    FAIL_TEMPLATE = "\n".join((
        "\n-----------",
        "Test Case {test_case}",
        "Output: {output}",
        "Failed: {message}",
        "credit: 0"
    ))

    credit = []
    totalCredit = 0

    fout = None

    @classmethod
    def _log(cls, s):
        (cls.fout or sys.stdout).write(s + '\n')

    def setUp(self):
        """Initialize test setup.
        """
        if studentExc:
            self.credit.append(0.0)
            self.results.append("exception on import: %s" % str(studentExc))
            raise studentExc

        self.student_submission = Submission(fout=self.__class__.fout)

    def tearDown(self):
        self.__class__.totalCredit = sum(self.__class__.credit)

    @classmethod
    def tearDownClass(cls):
        """Save student results at conclusion of test.
        """
        # Prints results after all tests complete
        for line in cls.results:
            cls._log(line)
        cls._log("\n====================")
        cls._log('Total Credit: {:.2f}'.format(cls.totalCredit))

    def check_results(self, params):

        error_message = ''

        to_box_score, to_zone_score = (0.0, 0.0)
        logmsg = ''

        if not self.student_submission.logmsgs.empty():
            logmsg = self.student_submission.logmsgs.get()

        if not self.student_submission.submission_score.empty():
            to_box_score, to_zone_score = self.student_submission.submission_score.get()

        if not self.student_submission.submission_error.empty():
            error_message = self.student_submission.submission_error.get()
            self.results.append(self.FAIL_TEMPLATE.format(message=error_message, output=logmsg, **params))
        else:
            self.results.append(
                self.SCORE_TEMPLATE.format(to_box_score=to_box_score,
                                           to_zone_score=to_zone_score,
                                           total_score=to_box_score + to_zone_score,
                                           output=logmsg,
                                           **params))

        self.credit.append(to_box_score + to_zone_score)

        self._log('test case {} credit: {}'.format(params['test_case'], to_box_score + to_zone_score))
        if error_message:
            self._log('{}'.format(error_message))

        self.assertFalse(error_message, error_message)

    def run_with_params(self, params):
        """Run test case using desired parameters.
        Args:
            params(dict): a dictionary of test parameters.
        """
        args = [
            params['test_case'],
            params['warehouse'],
            params['warehouse_cost'],
            params['robot_init'],
            params['robot_init2'],
            params['todo'],
            params['prob_as_intended'],
            params['correct_performed_actions' if DISPLAY_UNICODE_ARROWS else 'correct_performed_actions_alt_chars'],
            params['seed'],
        ]
        if DEBUGGING_SINGLE_PROCESS:
            self.student_submission.compare_student_policy(*args)
        else:
            test_process = mproc.Process(target=self.student_submission.compare_student_policy,
                                         args=args)

        if DEBUGGING_SINGLE_PROCESS:
            # Note: no TIMEOUT is checked in this case so that debugging isn't
            # inadvertently stopped
            self.check_results(params)
        else:
            logmsg = ''
            try:
                test_process.start()
                test_process.join(TIME_LIMIT)
            except Exception as exp:
                error_message = exp
            if test_process.is_alive():
                test_process.terminate()
                error_message = ('Test aborted due to timeout. ' +
                                 'Test was expected to finish in fewer than {} second(s).'.format(TIME_LIMIT))
                if not self.student_submission.logmsgs.empty():
                    logmsg = self.student_submission.logmsgs.get()
                self.results.append(self.FAIL_TEMPLATE.format(message=error_message, output=logmsg, **params))

            else:
                self.check_results(params)

    # def test_case_01(self):
    #     w = math.inf
    #     params = {'test_case': 1,
    #               'warehouse': ['1..',
    #                             '.#.',
    #                             '..@'],
    #               'warehouse_cost': [[13, 5, 6],
    #                                  [10, w, 2],
    #                                  [2, 11, 2]],
    #               'todo': ['1'],
    #               'robot_init': (2, 2),
    #               'robot_init2': (0, 1),
    #               'seed': 7638,
    #               'prob_as_intended': 70,
    #               'correct_performed_actions': {
    #                   'to_box': '↑↖+',
    #                   'to_zone': '↘-',
    #               },
    #               'correct_performed_actions_alt_chars': {
    #                   'to_box': '^`+',
    #                   'to_zone': '[-',
    #               },
    #               }
    #
    #     self.run_with_params(params)
    #
    #     # Notice that we have included several extra test cases below.
    #     # You can uncomment one or more of these for extra tests.
    #
    # def test_case_02(self):
    #     w = math.inf
    #     params = {'test_case': 2,
    #               'warehouse': ['1..',
    #                             '.#.',
    #                             '..@'],
    #               'warehouse_cost': [[13, 5, 6],
    #                                  [10, w, 2],
    #                                  [2, 11, 2]],
    #               'todo': ['1'],
    #               'robot_init': (2, 1),
    #               'robot_init2': (1, 0),
    #               'seed': 7638,
    #               'prob_as_intended': 20,
    #               'correct_performed_actions': {
    #                   'to_box': '↖+',
    #                   'to_zone': '↗↓↓↙↗↙→↘-',
    #               },
    #               'correct_performed_actions_alt_chars': {
    #                   'to_box': '`+',
    #                   'to_zone': '%vv]%]>[-',
    #               },
    #               }
    #
    #     self.run_with_params(params)
    #
    # def test_case_03(self):
    #     w = math.inf
    #     params = {'test_case': 3,
    #               'warehouse': ['##.####1',
    #                             '#.......',
    #                             '@.......'],
    #               'warehouse_cost': [[w, w, 3, w, w, w, w, 12],
    #                                  [w, 8, 10, 2, 10, 4, 15, 8],
    #                                  [15, 10, 10, 10, 7, 10, 2, 10]],
    #               'todo': ['1'],
    #               'robot_init': (0, 2),
    #               'robot_init2': (1, 7),
    #               'seed': 7638,
    #               'prob_as_intended': 75,
    #               'correct_performed_actions': {
    #                   'to_box': '↘↘↗↘↗+',
    #                   'to_zone': '↙↖↙↖↙↖-',
    #               },
    #               'correct_performed_actions_alt_chars': {
    #                   'to_box': '[[%[%+',
    #                   'to_zone': ']`]`]`-',
    #               },
    #               }
    #
    #     self.run_with_params(params)
    #
    # def test_case_04(self):
    #     w = math.inf
    #     params = {'test_case': 4,
    #               'warehouse': ['.........#..........',
    #                             '...#.....#..........',
    #                             '1..#................',
    #                             '...#................',
    #                             '....#....#####....##',
    #                             '......#..#..........',
    #                             '......#..#...@......'],
    #               'warehouse_cost': [[94, 56, 14, 0, 11, 74, 4, 85, 88, w, 10, 12, 93, 45, 30, 2, 3, 95, 2, 44],
    #                                  [82, 79, 61, w, 78, 59, 19, 11, 23, w, 91, 14, 1, 64, 62, 31, 8, 85, 69, 59],
    #                                  [0, 8, 76, w, 86, 11, 65, 74, 5, 34, 71, 8, 82, 38, 61, 45, 34, 31, 83, 25],
    #                                  [58, 67, 85, w, 2, 65, 9, 0, 42, 18, 90, 60, 84, 48, 21, 6, 9, 75, 63, 20],
    #                                  [9, 71, 27, 18, w, 3, 44, 93, 14, w, w, w, w, w, 67, 18, 85, 39, w, w],
    #                                  [58, 5, 53, 35, 84, 5, w, 22, 34, w, 19, 38, 19, 94, 59, 5, 72, 49, 92, 44],
    #                                  [63, 43, 74, 59, 60, 5, w, 95, 60, w, 76, 21, 56, 0, 93, 94, 66, 56, 37, 35]],
    #               'todo': ['1'],
    #               'robot_init': (6, 19),
    #               'robot_init2': (3, 1),
    #               'seed': 7638,
    #               'prob_as_intended': 80,
    #               'correct_performed_actions': {
    #                   'to_box': '↖↖↖←←↖↖↙↑↙↙↙↑↖↙↙↖↙↑←↖+',
    #                   'to_zone': '↘→↗↘↗→↗↘↘↗→↖↘→→↘↘↙-',
    #               },
    #               'correct_performed_actions_alt_chars': {
    #                   'to_box': '```<<``]^]]]^`]]`]^<`+',
    #                   'to_zone': '[>%[%>%[[%>`[>>[[]-',
    #               },
    #               }
    #
    #     self.run_with_params(params)
    #
    # def test_case_05(self):
    #     w = math.inf
    #     params = {'test_case': 5,
    #               'warehouse': ['.........#..........',
    #                             '####.....#..........',
    #                             '1..#................',
    #                             '...#................',
    #                             '....#....#####....##',
    #                             '......#..#..........',
    #                             '......#..#...@......'],
    #               'warehouse_cost': [[94, 56, 14, 0, 11, 74, 4, 85, 88, w, 10, 12, 93, 45, 30, 2, 3, 95, 2, 44],
    #                                  [w, w, w, w, 78, 59, 19, 11, 23, w, 91, 14, 1, 64, 62, 31, 8, 85, 69, 59],
    #                                  [0, 8, 76, w, 86, 11, 65, 74, 5, 34, 71, 8, 82, 38, 61, 45, 34, 31, 83, 25],
    #                                  [58, 67, 85, w, 2, 65, 9, 0, 42, 18, 90, 60, 84, 48, 21, 6, 9, 75, 63, 20],
    #                                  [9, 71, 27, 18, w, 3, 44, 93, 14, w, w, w, w, w, 67, 18, 85, 39, w, w],
    #                                  [58, 5, 53, 35, 84, 5, w, 22, 34, w, 19, 38, 19, 94, 59, 5, 72, 49, 92, 44],
    #                                  [63, 43, 74, 59, 60, 5, w, 95, 60, w, 76, 21, 56, 0, 93, 94, 66, 56, 37, 35]],
    #               'todo': ['1'],
    #               'robot_init': (0, 0),
    #               'robot_init2': (3, 1),
    #               'seed': 7638,
    #               'prob_as_intended': 75,
    #               'correct_performed_actions': {
    #                   'to_box': '→→→↘↘↙↙←↗↖+',
    #                   'to_zone': '↘→↗↘↗→↗↘↘↗→↖↘→→↘↘↙-',
    #               },
    #               'correct_performed_actions_alt_chars': {
    #                   'to_box': '>>>[[]]<%`+',
    #                   'to_zone': '[>%[%>%[[%>`[>>[[]-',
    #               },
    #               }
    #
    #     self.run_with_params(params)
    #
    # def test_case_06(self):
    #     w = math.inf
    #     params = {'test_case': 6,
    #               'warehouse': ['.........#..........',
    #                             '..##.....#..........',
    #                             '1..#................',
    #                             '...#................',
    #                             '....#....#####....##',
    #                             '......#..#..........',
    #                             '......#..#...@......'],
    #               'warehouse_cost': [[94, 56, 14, 0, 11, 74, 4, 85, 88, w, 10, 12, 93, 45, 30, 2, 3, 95, 2, 44],
    #                                  [1, 37, w, w, 78, 59, 19, 11, 23, w, 91, 14, 1, 64, 62, 31, 8, 85, 69, 59],
    #                                  [0, 8, 76, w, 86, 11, 65, 74, 5, 34, 71, 8, 82, 38, 61, 45, 34, 31, 83, 25],
    #                                  [58, 67, 85, w, 2, 65, 9, 0, 42, 18, 90, 60, 84, 48, 21, 6, 9, 75, 63, 20],
    #                                  [9, 71, 27, 18, w, 3, 44, 93, 14, w, w, w, w, w, 67, 18, 85, 39, w, w],
    #                                  [58, 5, 53, 35, 84, 5, w, 22, 34, w, 19, 38, 19, 94, 59, 5, 72, 49, 92, 44],
    #                                  [63, 43, 74, 59, 60, 5, w, 95, 60, w, 76, 21, 56, 0, 93, 94, 66, 56, 37, 35]],
    #               'todo': ['1'],
    #               'robot_init': (2, 19),
    #               'robot_init2': (3, 1),
    #               'seed': 7638,
    #               'prob_as_intended': 90,
    #               'correct_performed_actions': {
    #                   'to_box': '↙↖↙←←↖↖↙↖↙←↘↖↙←↙↖↙↑←↖+',
    #                   'to_zone': '↘→↗↘↗→↗↘→↗↗↗↙↘↘↘↙-',
    #               },
    #               'correct_performed_actions_alt_chars': {
    #                   'to_box': ']`]<<``]`]<[`]<]`]^<`+',
    #                   'to_zone': '[>%[%>%[>%%%][[[]-',
    #               },
    #               }
    #
    #     self.run_with_params(params)
    #
    # def test_case_07(self):
    #     w = math.inf
    #     params = {'test_case': 7,
    #               'warehouse': ['.........#..........',
    #                             '..##.....#..........',
    #                             '1..#................',
    #                             '...#................',
    #                             '....#....#####....##',
    #                             '......#..#..........',
    #                             '......#..#...@......'],
    #               'warehouse_cost': [[94, 56, 14, 0, 11, 74, 4, 85, 88, w, 10, 12, 93, 45, 30, 2, 3, 95, 2, 44],
    #                                  [1, 37, w, w, 78, 59, 19, 11, 23, w, 91, 14, 1, 64, 62, 31, 8, 85, 69, 59],
    #                                  [0, 8, 76, w, 86, 11, 65, 74, 5, 34, 71, 8, 82, 38, 61, 45, 34, 31, 83, 25],
    #                                  [58, 67, 85, w, 2, 65, 9, 0, 42, 18, 90, 60, 84, 48, 21, 6, 9, 75, 63, 20],
    #                                  [9, 71, 27, 18, w, 3, 44, 93, 14, w, w, w, w, w, 67, 18, 85, 39, w, w],
    #                                  [58, 5, 53, 35, 84, 5, w, 22, 34, w, 19, 38, 19, 94, 59, 5, 72, 49, 92, 44],
    #                                  [63, 43, 74, 59, 60, 5, w, 95, 60, w, 76, 21, 56, 0, 93, 94, 66, 56, 37, 35]],
    #               'todo': ['1'],
    #               'robot_init': (0, 10),
    #               'robot_init2': (3, 1),
    #               'seed': 7638,
    #               'prob_as_intended': 82,
    #               'correct_performed_actions': {
    #                   'to_box': '↘↙↙↖↙←↙↖↖↙←↙↑↑+',
    #                   'to_zone': '↘→↗↘↗→↗↘↘↗→↖↘→→↘↘↙-',
    #               },
    #               'correct_performed_actions_alt_chars': {
    #                   'to_box': '[]]`]<]``]<]^^+',
    #                   'to_zone': '[>%[%>%[[%>`[>>[[]-',
    #               },
    #               }
    #
    #     self.run_with_params(params)

    def test_case_08(self):
        w = math.inf
        params = {'test_case': 8,
                  'warehouse': ['............#...............',
                                '......#.....#...............',
                                '.....................#......',
                                '............................',
                                '..1...#.....................',
                                '............##########......',
                                '......#..#..#.........#.....',
                                '.........#..#....@....#.....',
                                '......#.....#.........#.....',
                                '............#.........#.....'],
                  'warehouse_cost': [
                      [94, 56, 14, 0, 11, 74, 4, 85, 88, 10, 12, 93, w, 45, 30, 2, 3, 95, 2, 44, 82, 79, 61, 78, 59, 19, 11, 23],
                      [91, 14, 1, 64, 62, 31, w, 8, 85, 69, 59, 8, w, 76, 86, 11, 65, 74, 5, 34, 71, 8, 82, 38, 61, 45, 34, 31],
                      [83, 25, 58, 67, 85, 2, 65, 9, 0, 42, 18, 90, 60, 84, 48, 21, 6, 9, 75, 63, 20, w, 9, 71, 27, 18, 3, 44],
                      [93, 14, 67, 18, 85, 39, 58, 5, 53, 35, 84, 5, 22, 34, 19, 38, 19, 94, 59, 5, 72, 49, 92, 44, 63, 43, 74, 59],
                      [60, 5, 95, 60, 76, 21, w, 56, 93, 94, 66, 56, 37, 35, 15, 94, 23, 53, 55, 93, 15, 67, 13, 62, 48, 84, 32, 82],
                      [24, 44, 13, 89, 89, 20, 74, 34, 19, 92, 41, 95, w, w, w, w, w, w, w, w, w, w, 57, 92, 9, 10, 50, 27],
                      [6, 36, 4, 28, 64, 11, w, 89, 40, w, 39, 58, w, 8, 74, 32, 9, 88, 54, 25, 12, 50, w, 24, 90, 58, 64, 30],
                      [46, 26, 65, 89, 53, 22, 74, 26, 38, w, 7, 45, w, 68, 19, 63, 93, 70, 60, 42, 17, 16, w, 6, 79, 21, 18, 69],
                      [8, 91, 41, 21, 0, 85, w, 86, 7, 81, 11, 92, w, 18, 27, 5, 55, 50, 94, 41, 26, 86, w, 48, 35, 68, 80, 38],
                      [54, 40, 87, 73, 19, 68, 11, 92, 33, 35, 52, 51, w, 72, 35, 67, 14, 89, 48, 35, 27, 38, w, 91, 75, 50, 6, 44]],
                  'todo': ['1'],
                  'robot_init': (9, 13),
                  'robot_init2': (3, 3),
                  'seed': 7638,
                  'prob_as_intended': 80,
                  'correct_performed_actions': {
                      'to_box': '↗→↘↗↗↗↘↗↘↗↑↙↑↗←↖↙↖↑↙↙↖↙↙←←←↖↙↖←↙↖↙←+',
                      'to_zone': '↗→↘→↗↘↗↘↓↗→↑↓↘↗↗→↗↓↘↘↘→↘↙↙↖←-',
                  },
                  'correct_performed_actions_alt_chars': {
                      'to_box': '%>[%%%[%[%^]^%<`]`^]]`]]<<<`]`<]`]<+',
                      'to_zone': '%>[>%[%[v%>^v[%%>%v[[[>[]]`<-',
                  },
                  }

        self.run_with_params(params)

    def test_case_09(self):
        w = math.inf
        params = {'test_case': 9,
                  'warehouse': ['............#...............',
                                '......#.....#...............',
                                '.....................#......',
                                '............................',
                                '......#.....................',
                                '............##########......',
                                '......#..#..#.........#.....',
                                '.........#..#....@....#.....',
                                '......#.....#....1....#.....',
                                '............#.........#.....']
            ,
                  'warehouse_cost': [
                      [94, 56, 14, 0, 11, 74, 4, 85, 88, 10, 12, 93, w, 45, 30, 2, 3, 95, 2, 44, 82, 79, 61, 78, 59, 19, 11, 23],
                      [91, 14, 1, 64, 62, 31, w, 8, 85, 69, 59, 8, w, 76, 86, 11, 65, 74, 5, 34, 71, 8, 82, 38, 61, 45, 34, 31],
                      [83, 25, 58, 67, 85, 2, 65, 9, 0, 42, 18, 90, 60, 84, 48, 21, 6, 9, 75, 63, 20, w, 9, 71, 27, 18, 3, 44],
                      [93, 14, 67, 18, 85, 39, 58, 5, 53, 35, 84, 5, 22, 34, 19, 38, 19, 94, 59, 5, 72, 49, 92, 44, 63, 43, 74, 59],
                      [60, 5, 95, 60, 76, 21, w, 56, 93, 94, 66, 56, 37, 35, 15, 94, 23, 53, 55, 93, 15, 67, 13, 62, 48, 84, 32, 82],
                      [24, 44, 13, 89, 89, 20, 74, 34, 19, 92, 41, 95, w, w, w, w, w, w, w, w, w, w, 57, 92, 9, 10, 50, 27],
                      [6, 36, 4, 28, 64, 11, w, 89, 40, w, 39, 58, w, 8, 74, 32, 9, 88, 54, 25, 12, 50, w, 24, 90, 58, 64, 30],
                      [46, 26, 65, 89, 53, 22, 74, 26, 38, w, 7, 45, w, 68, 19, 63, 93, 70, 60, 42, 17, 16, w, 6, 79, 21, 18, 69],
                      [8, 91, 41, 21, 0, 85, w, 86, 7, 81, 11, 92, w, 18, 27, 5, 55, 50, 94, 41, 26, 86, w, 48, 35, 68, 80, 38],
                      [54, 40, 87, 73, 19, 68, 11, 92, 33, 35, 52, 51, w, 72, 35, 67, 14, 89, 48, 35, 27, 38, w, 91, 75, 50, 6, 44],
                  ],
                  'todo': ['1'],
                  'robot_init': (0, 0),
                  'robot_init2': (7, 18),
                  'seed': 7638,
                  'prob_as_intended': 80,
                  'correct_performed_actions': {
                      'to_box': '↘→↗→↘↗↘↘↙↗↘↖↓↘→↗→→↓↗↗↘↗↗→↘↘↘↓↙↙↖↙+',
                      'to_zone': '-',
                  },
                  'correct_performed_actions_alt_chars': {
                      'to_box': '[>%>[%[[]%[`v[>%>>v%%[%%>[[[v]]`]+',
                      'to_zone': '-',
                  },
                  }
        self.run_with_params(params)

    def test_case_10(self):
        w = math.inf
        params = {'test_case': 10,
                  'warehouse': ['........',
                                '.######.',
                                '.@.1....'],
                  'warehouse_cost': [[1, 1, 1, 1, 1, 1, 1, 1],
                                     [1, w, w, w, w, w, w, 1],
                                     [1, 95, 95, 1, 1, 1, 1, 1]],
                  'todo': ['1'],
                  'robot_init': (2, 0),
                  'robot_init2': (2, 4),
                  'seed': 7638,
                  'prob_as_intended': 99,
                  'correct_performed_actions': {
                      'to_box': '↑↗→→→→→↘↙←←+',
                      'to_zone': '→→↗↖←←←←←↙-',
                  },
                  'correct_performed_actions_alt_chars': {
                      'to_box': '^%>>>>>[]<<+',
                      'to_zone': '>>%`<<<<<]-',
                  },
                  }

        self.run_with_params(params)


# Only run all of the test automatically if this file was executed from the command line.
# Otherwise, let Nose/py.test do it's own thing with the test cases.
if __name__ == "__main__":
    if studentExc:
        print(studentExc)
        print('score: 0')
    else:
        student_id = who_am_i()
        if student_id:
            PartCTestCase.fout = sys.stdout
            unittest.main()
        else:
            print("Student ID not specified.  Please fill in 'whoami' variable.")
            print('score: 0')
