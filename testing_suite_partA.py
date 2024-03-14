
import unittest
import multiprocessing as mproc
import traceback
import sys
import copy
import io
from state import State

try:
    from warehouse import DeliveryPlanner_PartA, who_am_i
    studentExc = None
except Exception:
    studentExc = traceback.format_exc()
    
########################################################################
# For debugging this flag can be set to True to print state 
# which could result in a timeout
########################################################################
VERBOSE_FLAG = False

########################################################################
# For testing actions in the warehouse via keyboard input
# use the number pad as directional input
#        shift + <direction> will lift a box
# ctrl/command + <direction> will put a box down
# Note that this will also enable VISUALIZE_FLAG
########################################################################
TEST_MODE = False

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
VISUALIZE_FLAG = True if TEST_MODE else VISUALIZE_FLAG
if VISUALIZE_FLAG:
    from visualizer import GUI
DEBUGGING_SINGLE_PROCESS = True if VISUALIZE_FLAG else DEBUGGING_SINGLE_PROCESS

def truncate_output( s, max_len = 2000 ):
    if len(s) > max_len:
        return s[:max_len-70] + "\n***************** OUTPUT TRUNCATED DUE TO EXCESSIVE LENGTH!**************\n"
    else:
        return s

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
            self.submission_error = queue.Queue(1)
            self.logmsgs = queue.Queue(1)
        else:
            self.submission_score = mproc.Manager().Queue(1)
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
            
    def execute_student_plan(self, test_case, warehouse, boxes_todo):
        """Execute student plan and store results in submission.

        Args:
            warehouse(list(list)): the warehouse map to test against.
            boxes_todo(list): the order of boxes to deliver.
        """
        self._reset()

        state = State(warehouse)

        try:
            if TEST_MODE:
                action_list = ['user_input']
            else:
                student_planner = DeliveryPlanner_PartA(copy.deepcopy(warehouse), copy.deepcopy(boxes_todo))
                action_list = student_planner.plan_delivery(debug=VERBOSE_FLAG)

            num_delivered = 0
            next_box_to_deliver = boxes_todo[num_delivered]

            if VISUALIZE_FLAG:
                gui = GUI('A', test_case, state, len(action_list), TEST_MODE=TEST_MODE)
                quit_signal = gui.quit_signal
                if quit_signal:
                    self.log('GUI received quit signal before executing any actions.')
                    action_list = []
                prev_loc = state.robot_position
                prev_box_locs = copy.deepcopy(state.boxes)

            while action_list:
                action = gui.selected_action if TEST_MODE else action_list.pop(0)

                if VERBOSE_FLAG:
                    state.print_to_console( self.fout )
                    #state.print_to_console( )

                state.update_according_to(action)

                if VISUALIZE_FLAG:
                    gui.update(state, action, prev_loc, prev_box_locs)
                    quit_signal = gui.quit_signal
                    if quit_signal:
                        self.log('GUI received quit signal.')
                        break
                    prev_loc = state.robot_position
                    prev_box_locs = copy.deepcopy(state.boxes)
                # check if new box has been delivered
                delivered = state.get_boxes_delivered()
                if len(delivered) > num_delivered:
                    last_box_delivered = delivered[-1]
                    if last_box_delivered == next_box_to_deliver:
                        num_delivered += 1
                        if num_delivered < len(boxes_todo):
                            next_box_to_deliver = boxes_todo[num_delivered]
                        else:
                            # all boxes delivered: end test
                            break
                    else:
                        # wrong box delivered: kill test
                        raise Exception('wrong box delivered: {} instead of {}'.format(last_box_delivered,
                                                                                       next_box_to_deliver))

            if VERBOSE_FLAG:
                # print final state
                self.log('\n\n')
                self.log('Final State: ')
                state.print_to_console( self.fout )

            #Note, to receive credit, you must have deliverd all boxes.
            if num_delivered == len(boxes_todo):
               self.submission_score.put(state.get_total_cost())
            else:
               self.submission_score.put(float('inf')) 

        except:
            self.submission_error.put(traceback.format_exc())
            self.submission_score.put(float('inf'))

        self.logmsgs.put( truncate_output( self.fout.getvalue() ) )


class PartATestCase(unittest.TestCase):
    """ Test Part A.
    """

    results = ['', 'PART A TEST CASE RESULTS']
    SCORE_TEMPLATE = "\n".join((
        "\n-----------",
        "Test Case {test_case}",
        "Output: {output}",
        "cost: {cost}  (benchmark cost {benchmark_cost})",
        "credit: {score:.2f}"
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
        (cls.fout or sys.stdout).write( s + '\n')

    def setUp(self):
        """Initialize test setup.
        """
        if studentExc:
            self.credit.append( 0.0 )
            self.results.append( "exception on import: %s" % str(studentExc) )
            raise studentExc

        self.student_submission = Submission( fout = self.__class__.fout )

    def tearDown(self):
        self.__class__.totalCredit = sum(self.__class__.credit)

    @classmethod
    def tearDownClass(cls):
        """Save student results at conclusion of test.
        """
        # Prints results after all tests complete
        for line in cls.results:
            cls._log(line)
        cls._log("\n-----------")
        cls._log('\nTotal Credit: {:.2f}'.format(cls.totalCredit))


    def check_results(self, params):

        error_message = ''
        cost = float('inf')
        score = 0.0
        logmsg = ''

        if not self.student_submission.logmsgs.empty():
            logmsg = self.student_submission.logmsgs.get()

        if not self.student_submission.submission_score.empty():
            cost = self.student_submission.submission_score.get()

        score = float(params['benchmark_cost']) / float(cost)

        if not self.student_submission.submission_error.empty():
            error_message = self.student_submission.submission_error.get()
            self.results.append(self.FAIL_TEMPLATE.format(message=error_message, output = logmsg, **params))

        else:
            self.results.append(self.SCORE_TEMPLATE.format(cost=cost, score=score, output = logmsg, **params))

        self.credit.append(score)

        self._log('test case {} credit: {}'.format(params['test_case'], score))
        if error_message:
            self._log('{}'.format(error_message))

        self.assertFalse(error_message, error_message)

    def run_with_params(self, params):
        """Run test case using desired parameters.

        Args:
            params(dict): a dictionary of test parameters.
        """

        if DEBUGGING_SINGLE_PROCESS:
            self.student_submission.execute_student_plan(params['test_case'], params['warehouse'], params['todo'])
        else:
            test_process = mproc.Process(target=self.student_submission.execute_student_plan, args=(params['test_case'],
                                                                                                    params['warehouse'],
                                                                                                    params['todo']))

        if DEBUGGING_SINGLE_PROCESS :

            # Note: no TIMEOUT is checked in this case so that debugging isn't 
            # inadvertently stopped

            self.check_results( params )

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
                self.results.append(self.FAIL_TEMPLATE.format(message=error_message, output = logmsg, **params))

            else:

                self.check_results( params )




    def test_case_01(self):
        params = {'test_case': 1,
                  'warehouse': ['......',
                                '......',
                                '....#.',
                                '....#.',
                                '..##1#',
                                '...32@'],
                  'todo': ['1', '2', '3'],
                  'benchmark_cost': 67}

        self.run_with_params(params)

    # def test_case_01(self):
    #     params = {'test_case': 1,
    #               'warehouse': ['1#2',
    #                             '.#.',
    #                             '..@'],
    #               'todo': ['1', '2'],
    #               'benchmark_cost': 23}
    #
    #     self.run_with_params(params)
    #
    # # Notice that we have included several extra test cases below.
    # # You can uncomment one or more of these for extra tests.
    #
    def test_case_02(self):
        params = {'test_case': 2,
                  'warehouse': ['@...J1'],
                  'todo': ['J', '1'],
                  'benchmark_cost': 34}

        self.run_with_params(params)

    def test_case_03(self):
        params = {'test_case': 3,
                  'warehouse': ['1.#@#.4',
                                '2#.#.#3'],
                  'todo': ['1', '2', '3', '4'],
                  'benchmark_cost': 57}

        self.run_with_params(params)

    # *** CREDIT TO: Kowsalya Subramanian for adding this test case
    def test_case_04(self):
        params = {'test_case': 4,
                  'warehouse': ['3#@',
                                '2#.',
                                '1..'],
                  'todo': ['1', '2', '3'],
                  'benchmark_cost': 44}

        self.run_with_params(params)

    # *** CREDIT TO: Gideon Rossman for adding this test case
    def test_case_05(self):
        params = {'test_case': 5,
                  'warehouse': ['..1.',
                                '..@.',
                                '....',
                                '2...'],
                  'todo': ['1', '2'],
                  'benchmark_cost': 19}

        self.run_with_params(params)

    # # *** CREDIT TO: venkatasatyanarayana kamisetti for adding this test case
    def test_case_06(self):
        params = {'test_case': 6,
                  'warehouse': ['1..',
                                '...',
                                '@.2'],
                  'todo': ['1', '2'],
                  'benchmark_cost': 16}

        self.run_with_params(params)

    # # *** CREDIT TO: Dana Johnson for adding this test case
    def test_case_07(self):
        params = {'test_case': 7,
                  'warehouse': ['#5######',
                                '#I#234J#',
                                '#H#1##6#',
                                '#G#0@#7#',
                                '#F####8#',
                                '#EDCBA9#',
                                '########'],
                  'todo': ['0', '1', '2', '3', '4', 'J', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H',
                           'I', '5'],
                  'benchmark_cost': 636}

        self.run_with_params(params)

    # *** CREDIT TO: Dana Johnson for adding this test case
    def test_case_08(self):
        params = {'test_case': 8,
                  'warehouse': ['#######2',
                                '#......1',
                                '#@......'],
                  'todo': ['1', '2'],
                  'benchmark_cost': 47}

        self.run_with_params(params)

    def test_case_09(self):
        params = {'test_case': 9,
                  'warehouse': ['..#1..',
                                '......',
                                '..####',
                                '..#2.#',
                                '.....@'],
                  'todo': ['1', '2'],
                  'benchmark_cost': 43}

        self.run_with_params(params)

    # Test Case 10
    def test_case_10(self):
        params = {'test_case': 10,
                  'warehouse': ['..#1..',
                                '#....#',
                                '..##.#',
                                '..#2.#',
                                '#....@'],
                  'todo': ['1', '2'],
                  'benchmark_cost': 30}

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
            PartATestCase.fout = sys.stdout
            unittest.main()
        else:
            print("Student ID not specified.  Please fill in 'whoami' variable.")
            print('score: 0')
