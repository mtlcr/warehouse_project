
import time
from os import environ
environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
environ['SDL_VIDEO_CENTERED'] = '1'

import pygame

from state import DIRECTIONS, DIRECTION_INDICES, DELTA_DIRECTIONS

from pygame.locals import (
    K_SPACE,
    K_m,
    KEYDOWN,
    QUIT,
    K_ESCAPE,
    K_z,K_1,K_KP_1,
    K_x,K_2,K_KP_2,
    K_c,K_3,K_KP_3,
    K_a,K_4,K_KP_4,
    K_d,K_6,K_KP_6,
    K_q,K_7,K_KP_7,
    K_w,K_8,K_KP_8,
    K_e,K_9,K_KP_9,
)

GAME_KEYS = {
    K_SPACE: 'pause',
    K_m: 'mask',
    K_ESCAPE: 'quit',
    K_KP_1: 'sw', K_1: 'sw', K_z: 'sw',
    K_KP_2: 's', K_2: 's', K_x: 's',
    K_KP_3: 'se', K_3: 'se', K_c: 'se',
    K_KP_4: 'w', K_4: 'w', K_a: 'w',
    K_KP_6: 'e', K_6: 'e', K_d: 'e',
    K_KP_7: 'nw', K_7: 'nw', K_q: 'nw',
    K_KP_8: 'n', K_8: 'n', K_w: 'n',
    K_KP_9: 'ne', K_9: 'ne', K_e: 'ne',
}

FRAME_RATE_PER_SECOND = 1
# Speed Reference (ymmv)
# 0: 'MANUAL-PAUSE'
# 1: 'SNAIL'
# 2: 'TURTLE'
# 3: 'HARE'
# 4: 'CHEETAH'
# 5: 'FALCON'

PAUSE_AT_END_FOR_X_SECONDS = 1

# colors needed for the gui background/text
BLACK = pygame.Color('Black')
WHITE = pygame.Color('White')
BOX_BROWN = pygame.Color(217, 148, 78)

IMGS = {
    'wall'              : 'viz/wall.png',
    'traversable'       : 'viz/traversable.png',
    'robot'             : 'viz/robot.png',
    'robot_lift'        : 'viz/robot_lift.png',
    'robot_down'        : 'viz/robot_down.png',
    'robot_with_box'    : 'viz/robot_with_box.png',
    'box'               : 'viz/box.png',
    'dropzone'          : 'viz/dropzone.png',
    'jay'               : 'viz/jaybot.png',
    'jay_lift'          : 'viz/jaybot_lift.png',
    'jay_down'          : 'viz/jaybot_down.png',
    'mask'              : 'viz/mask.png',
    'illegal_move'      : 'viz/red_x.png',
    'move ne'           : 'viz/ne.png',
    'move e'            : 'viz/e.png',
    'move se'           : 'viz/se.png',
    'move s'            : 'viz/s.png',
    'move sw'           : 'viz/sw.png',
    'move w'            : 'viz/w.png',
    'move nw'           : 'viz/nw.png',
    'move n'            : 'viz/n.png',
    'down'              : 'viz/down.png',
    'lift'              : 'viz/lift.png',
}

WAREHOUSE_LEGEND = {
    '*'         : 'robot',
    '*^'        : 'robot_lift',
    '*v'        : 'robot_down',
    '#'         : 'wall',
    '.'         : 'traversable',
    '@'         : 'dropzone',
    '*+box'     : 'robot_with_box',
    '!'         : 'illegal_move',
    'move ne'   : 'move ne',
    'move e'    : 'move e',
    'move se'   : 'move se',
    'move s'    : 'move s',
    'move sw'   : 'move sw',
    'move w'    : 'move w',
    'move nw'   : 'move nw',
    'move n'    : 'move n',
    'down'      : 'down',
    'lift'      : 'lift',
}

ACTION_LEGEND = {
    'lift': '^',
    'down': 'v',
}

########################################################################
# If you're here, good job on reading/looking at the code!
# Here's an easter egg for your efforts (there may be more).
# This flag was introduced a while ago, hopefully it won't
# be necessary much longer.  Toggle for permanent safety or
# use the keyboard shortcut m while the visualization is running.
########################################################################
MASK_FLAG = False


class GUI:
    BORDER = 50
    CELL_HEIGHT = 100
    CELL_WIDTH = 100
    FONT_SIZE = 20
    CELL_SIZE = CELL_HEIGHT, CELL_WIDTH
    ILLEGAL_MOVE_PENALTY = 100
    MAX_WINDOW_SIZE = .80   # percent of full screen as decimal
    pygame.init()
    FONT = pygame.font.Font('freesansbold.ttf', FONT_SIZE)

    display_info = pygame.display.Info()
    SCREEN_H = display_info.current_h
    SCREEN_W = display_info.current_w

    def __init__(self, part, test_case, state, total_num_actions, TEST_MODE=False):
        pygame.display.init()

        self.part = part
        self.test_case = test_case
        self.total_actions_left = (total_num_actions+1) if not TEST_MODE else 1
        self.grid = state.warehouse_state
        self.boxes_delivered = []
        self.grid_num_rows = len(self.grid)
        self.grid_num_cols = len(self.grid[0])
        self.prev_illegal_move = False
        self.cost_so_far = 0
        self.selected_action = None
        self.quit_signal = False
        self.TEST_MODE = TEST_MODE

        max_w = ((self.MAX_WINDOW_SIZE * self.SCREEN_W) // 100) * 100
        max_h = ((self.MAX_WINDOW_SIZE * self.SCREEN_H) // 100) * 100
        max_rows = max_h / self.CELL_HEIGHT
        max_cols = max_w / self.CELL_WIDTH

        # dynamically shrink grid if too large for screen
        if self.grid_num_rows > max_rows or self.grid_num_cols > max_cols:
            cell_w = int(max_w // self.grid_num_cols)
            cell_h = int(max_h // self.grid_num_rows)
            min_dimension = min(cell_w, cell_h)
            self.CELL_HEIGHT = min_dimension
            self.CELL_WIDTH = min_dimension
            self.FONT_SIZE = int(self.FONT_SIZE * (min_dimension/100))
            self.FONT = pygame.font.Font('freesansbold.ttf', self.FONT_SIZE)

        self.screen_height = (self.grid_num_rows * self.CELL_HEIGHT) + (2 * self.BORDER)
        self.screen_width = (self.grid_num_cols * self.CELL_WIDTH) + (2 * self.BORDER)
        self.CELL_SIZE = self.CELL_HEIGHT, self.CELL_WIDTH
        self.screen = pygame.display.set_mode([self.screen_width, self.screen_height])

        self.update(state, 'START')

    def update(self, state, action, prev_loc=None, prev_box_locs=None):
        global MASK_FLAG
        self.prev_box_locs = prev_box_locs
        self.state = state
        self.total_actions_left -= 1
        self.grid = state.warehouse_state
        self.warehouse_cost = state.warehouse_cost
        box_held = state.box_held
        dropzone = state.dropzone
        self.robot_position = robot_position = state.robot_position
        total_cost = state.total_cost
        failed_action = False

        delivered_box = len(state.boxes_delivered) > len(self.boxes_delivered)
        self.boxes_delivered = state.boxes_delivered[:]

        if 'move' in action or 'down' in action:
            _, self.direction = action.split()
        if 'lift' in action:
            if not self.prev_illegal_move:
                _, box = action.split()
                self.direction = self.get_direction(self.robot_position, box)

        self.screen.fill(WHITE)
        self.update_bot()

        # draw warehouse sprites
        for r in range(self.grid_num_rows):
            for c in range(self.grid_num_cols):
                x = self.BORDER + c * self.CELL_WIDTH
                y = self.BORDER + r * self.CELL_HEIGHT
                grid_symbol = self.grid[r][c]
                val = self.warehouse_cost[r][c]
                self.draw_sprite('.', x, y, val)
                action = 'lift' if 'lift' in action else 'down' if 'down' in action else action

                if grid_symbol == '*':
                    if robot_position == dropzone:
                        self.draw_sprite('@', x, y)
                    if box_held:
                        grid_symbol += '+box'
                    elif action in ('lift', 'down'):
                        grid_symbol+=ACTION_LEGEND[action]

                self.draw_sprite(grid_symbol, x, y, val)

                # detect if illegal move was performed and display visual indicator
                if '*' in grid_symbol and self.cost_so_far + self.ILLEGAL_MOVE_PENALTY <= total_cost and action!='START':
                    self.draw_sprite('!', x, y)
                    failed_action = True
                if grid_symbol == '@' and delivered_box:
                    self.draw_sprite('remove_box', x, y)

                # draw visual aid indicating action taken from previous location
                if (r, c) == prev_loc:
                    if action in ('lift', 'down'):
                        self.draw_sprite(WAREHOUSE_LEGEND[action], x, y, meta=action)
                        self.draw_sprite(WAREHOUSE_LEGEND[f'move {self.direction}'], x, y, meta=None)
                    else:
                        self.draw_sprite(WAREHOUSE_LEGEND[action], x, y, meta=None)
        self.cost_so_far = total_cost

        # draw game msgs
        test_case_msg = f'Part: {self.part}'
        text = self.FONT.render(test_case_msg, True, BLACK)
        self.screen.blit(text, (self.BORDER, self.screen_height - self.BORDER))

        test_case_msg = f'Test Case # {self.test_case}'
        text = self.FONT.render(test_case_msg, True, BLACK)
        self.screen.blit(text, (self.BORDER, self.BORDER - self.BORDER // 2))

        cost_msg = f'Cost: {total_cost}'
        text = self.FONT.render(cost_msg, True, BLACK)
        self.screen.blit(text, (self.BORDER + 200, self.BORDER - self.BORDER // 2))

        delivered_msg = f'Delivered: {self.boxes_delivered}'
        text = self.FONT.render(delivered_msg, True, BLACK)
        self.screen.blit(text, (self.BORDER, self.screen_height - self.BORDER + 20))

        action_msg = f'Move Count: {self.total_actions_left * (1 if not self.TEST_MODE else -1) }'
        action_msg += f' [illegal: {action}]' if failed_action else ''
        text = self.FONT.render(action_msg, True, BLACK)
        self.screen.blit(text, (self.BORDER, 0))

        # render new display
        pygame.display.update()

        # game event loop
        pause = True if (FRAME_RATE_PER_SECOND == 0 or self.TEST_MODE or self.selected_action=='pause') else False
        quit_signal = False
        game_play = True
        while game_play:
            game_play = pause
            for event in pygame.event.get():
                if event.type == KEYDOWN:

                    if event.key not in GAME_KEYS:
                        continue
                    user_input = GAME_KEYS.get(event.key)

                    if event.key == K_SPACE:
                        if self.TEST_MODE:
                            continue
                        pause = False
                        self.selected_action = 'pause' if self.selected_action!='pause' else None
                        break
                    if event.key == K_ESCAPE:
                        game_play = False
                        quit_signal = True
                        break
                    elif event.key == K_m:
                        MASK_FLAG = not MASK_FLAG
                        break

                    if self.TEST_MODE:
                        action_type = None
                        if event.mod & pygame.KMOD_CTRL:
                            action_type = 'down'
                        elif event.mod & pygame.KMOD_SHIFT:
                            action_type = 'lift'
                        if action_type in ('lift','down'):
                            is_legal_move, destination_status = self.check_user_input(action_type, user_input)
                            if is_legal_move:
                                self.direction = user_input
                                self.selected_action = f'{action_type} {destination_status if action_type=="lift" else user_input}'
                            else:
                                self.prev_illegal_move = True
                                self.direction = user_input
                                self.selected_action = f'{action_type} {self.direction}'
                        else:
                            self.selected_action = f'move {user_input}'
                        game_play=False
                        break
                if event.type == QUIT:
                    pause = False
                    quit_signal = True
        # initiate wait
        if FRAME_RATE_PER_SECOND and not self.TEST_MODE:
            time.sleep( 1 / FRAME_RATE_PER_SECOND )

        if self.total_actions_left == 0 and not self.TEST_MODE:
            time.sleep(PAUSE_AT_END_FOR_X_SECONDS)

        self.quit_signal = quit_signal


    def check_user_input(self, action_type, direction):
        if action_type == 'lift':
            dest_x, dest_y = destination = (DELTA_DIRECTIONS[DIRECTION_INDICES[direction]][0] + self.robot_position[0],
                                            DELTA_DIRECTIONS[DIRECTION_INDICES[direction]][1] + self.robot_position[1])

            destination_is_adjacent = self.state._are_adjacent(self.robot_position, destination)
            destination_is_within_warehouse = self.state._is_within_warehouse(destination)
            destination_status = self.grid[dest_x][dest_y] if destination_is_within_warehouse else 'None!'
            box_exists = destination_status.isalnum()

            is_legal_move = (destination_is_adjacent and box_exists and destination_is_within_warehouse)

        if action_type == 'down':
            dest_x, dest_y = destination = (DELTA_DIRECTIONS[DIRECTION_INDICES[direction]][0] + self.robot_position[0],
                                            DELTA_DIRECTIONS[DIRECTION_INDICES[direction]][1] + self.robot_position[1])

            destination_is_adjacent = self.state._are_adjacent(self.robot_position, destination)
            destination_is_within_warehouse = self.state._is_within_warehouse(destination)
            destination_status = self.grid[dest_x][dest_y] if destination_is_within_warehouse else 'None!'
            clear_path = destination_status in '.@'

            is_legal_move = (destination_is_adjacent and clear_path and destination_is_within_warehouse)

        return is_legal_move, destination_status


    def get_direction(self, robot_position, box):
        box_location = self.prev_box_locs[box]
        delta = box_location[0] - robot_position[0], box_location[1] - robot_position[1]
        delta_index = DELTA_DIRECTIONS.index(delta)
        direction = DIRECTIONS[delta_index]
        return direction


    def draw_sprite(self, key, x, y, meta=None):
        key_val = WAREHOUSE_LEGEND.get(key, 'box')
        img = pygame.image.load(IMGS[key_val])

        if key == 'remove_box':
            img = pygame.transform.scale(img, [i // 2 for i in self.CELL_SIZE]).convert_alpha()
            img.set_alpha(200)
            x += self.CELL_WIDTH // 4
            y += self.CELL_HEIGHT // 4
        else:
            if meta in ('lift', 'down'):
                img = pygame.transform.scale(img, [i // 3 for i in self.CELL_SIZE]).convert_alpha()
                x += self.CELL_WIDTH // 3
                y += self.CELL_WIDTH // 2
            else:
                img = pygame.transform.scale(img, self.CELL_SIZE).convert_alpha()

        self.screen.blit(img, (x, y))

        # safety first
        if '*' in key and MASK_FLAG and not self.state.box_held:
            img = pygame.image.load(IMGS['mask'])
            img = pygame.transform.scale(img, (self.CELL_WIDTH // 2, self.CELL_HEIGHT // 5)).convert_alpha()
            x += self.CELL_WIDTH // 4
            y += int(self.CELL_HEIGHT * .35)
            self.screen.blit(img, (x, y))

        # label box ID
        if key_val == 'box' and key != 'remove_box':
            text = self.FONT.render(key, True, BOX_BROWN)
            self.screen.blit(text, (x + self.CELL_WIDTH // 5, y + self.CELL_HEIGHT // 6))

        if key_val == 'traversable' and self.part !='A':
            text = self.FONT.render(str(meta), True, BLACK)
            self.screen.blit(text, (x + self.CELL_WIDTH // 2.5, y + self.CELL_HEIGHT // 2.5))


    def update_bot(self):
        if self.boxes_delivered and self.boxes_delivered[-1] == 'J':
            WAREHOUSE_LEGEND['*'] = 'jay'
            WAREHOUSE_LEGEND['*^'] = 'jay_lift'
            WAREHOUSE_LEGEND['*v'] = 'jay_down'
        else:
            WAREHOUSE_LEGEND['*'] = 'robot'
            WAREHOUSE_LEGEND['*^'] = 'robot_lift'
            WAREHOUSE_LEGEND['*v'] = 'robot_down'


    def quit(self):
        pygame.display.quit()
        return 'Quit!'
