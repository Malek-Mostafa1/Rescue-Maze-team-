import time
from collections import namedtuple
from enum import IntEnum
from src.shared import ray_buffer
from src.arduino_handler import send_command

Tile = namedtuple('Tile', ['x', 'y'])

class TileState(IntEnum):
    UNVISITED = 0
    VISITED = 1
    BLACK = 2
    RED = 3

class Direction(IntEnum):
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3

class DFSMazeExplorer:
    def __init__(self, width: int, height: int, start_x: int = 0, start_y: int = 0):
        self.width = width
        self.height = height
        self.start_x = start_x
        self.start_y = start_y
        
        self.map = {}
        self.walls = {}
        self.victims = {}
        self.colors = {}
        
        self._ensure_tile_exists(start_x, start_y)
        self.stack = []
        
        self.current_x = start_x
        self.current_y = start_y
        self.current_heading = Direction.NORTH
        
        self.exploration_complete = False
        self.total_tiles_visited = 0
        
        self.motor_controller = None
        self.lidar = None

    def set_hardware(self, motor_controller, lidar_adapter):
        self.motor_controller = motor_controller
        self.lidar = lidar_adapter
    
    def _ensure_tile_exists(self, x, y):
        if x not in self.map:
            self.map[x] = {}
            self.walls[x] = {}
            self.victims[x] = {}
            self.colors[x] = {}
        if y not in self.map[x]:
            self.map[x][y] = TileState.UNVISITED
            self.walls[x][y] = {'N': False, 'E': False, 'S': False, 'W': False}
            self.victims[x][y] = None
            self.colors[x][y] = None
    
    def _is_visited(self, x, y):
        self._ensure_tile_exists(x, y)
        return self.map[x][y] == TileState.VISITED

    def get_walls(self):
        if not self.lidar:
            return {'N': False, 'E': False, 'S': False, 'W': False}
        
        wall_threshold = 27 
        rays = self.lidar.get_rays()
        
        wall_data = {
            'N': (rays['N'] >= 5) and (rays['N'] < wall_threshold),
            'E': (rays['E'] >= 5) and (rays['E'] < wall_threshold),
            'S': (rays['S'] >= 5) and (rays['S'] < wall_threshold),
            'W': (rays['W'] >= 5) and (rays['W'] < wall_threshold),
        }
        
        print(f"[WALLS] ({self.current_x},{self.current_y}) heading={self.current_heading.name}")
        print(f"[WALLS] Distances - N:{rays['N']:.1f} E:{rays['E']:.1f} S:{rays['S']:.1f} W:{rays['W']:.1f}")
        print(f"[WALLS] Detected - {wall_data}")
        return wall_data
    
    def check_tile_color(self):
        from src.color_handler import check_tile_color
        color = check_tile_color()
        if color:
            print(f"[COLOR] Detected {color} at ({self.current_x}, {self.current_y})")
        return color

    def scan_for_victims(self):
        victim = self.motor_controller.scan_for_victims() if self.motor_controller else None
        if victim:
            self.victims[self.current_x][self.current_y] = victim
        return victim

    def get_direction_offset(self, direction: Direction):
        offsets = {
            Direction.NORTH: (0, 1),
            Direction.EAST: (1, 0),
            Direction.SOUTH: (0, -1),
            Direction.WEST: (-1, 0),
        }
        return offsets[direction]
    
    def get_wall_key_for_direction(self, direction: Direction):
        keys = {
            Direction.NORTH: 'N',
            Direction.EAST: 'E',
            Direction.SOUTH: 'S',
            Direction.WEST: 'W',
        }
        return keys[direction]

    def get_unvisited_neighbors(self):
        unvisited = []
        for direction in Direction:
            dx, dy = self.get_direction_offset(direction)
            nx, ny = self.current_x + dx, self.current_y + dy
            wall_key = self.get_wall_key_for_direction(direction)
            
            if self.walls[self.current_x][self.current_y][wall_key]:
                print(f"[WALL] Blocked {direction.name}")
                continue
            
            self._ensure_tile_exists(nx, ny)
            if self.map[nx][ny] == TileState.BLACK:
                print(f"[BLACK] ({nx},{ny}) is unreachable")
                continue
            
            if not self._is_visited(nx, ny):
                unvisited.append(direction)
                print(f"[UNVISITED] {direction.name} -> ({nx},{ny})")
            else:
                print(f"[VISITED] ({nx},{ny}) already visited")
        
        return unvisited

    def get_next_neighbor(self, unvisited):
        if not unvisited:
            return None
        
        priority_order = [
            self.current_heading,
            Direction((self.current_heading + 1) % 4),  # Right
            Direction((self.current_heading + 3) % 4),  # Left
            Direction((self.current_heading + 2) % 4),  # Back
        ]
        
        for preferred_dir in priority_order:
            if preferred_dir in unvisited:
                return preferred_dir
        
        return unvisited[0]

    def move_to(self, target_x: int, target_y: int, target_heading: Direction):
        if not self.motor_controller:
            return False
        
        delta = (target_heading - self.current_heading) % 4
        
        if delta == 0:
            result = self.motor_controller.move_forward_one_tile()
        elif delta == 1:
            if not self.motor_controller.turn_relative(1):
                return False
            result = self.motor_controller.move_forward_one_tile()
        elif delta == 3:
            if not self.motor_controller.turn_relative(-1):
                return False
            result = self.motor_controller.move_forward_one_tile()
        elif delta == 2:
            if not self.motor_controller.turn_relative(2):
                return False
            result = self.motor_controller.move_forward_one_tile()
        else:
            return False
        
        self._last_move_result = result
        return result

    def mark_current_tile_visited(self):
        self._ensure_tile_exists(self.current_x, self.current_y)
        self.map[self.current_x][self.current_y] = TileState.VISITED
        self.total_tiles_visited += 1
        print(f"[VISITED] ({self.current_x},{self.current_y}) total={self.total_tiles_visited}")

    def backtrack(self):
        if not self.stack:
            print("[BACKTRACK] Stack empty - exploration complete")
            return False
        
        prev_x, prev_y = self.stack.pop()
        print(f"[BACKTRACK] Returning to ({prev_x},{prev_y}) from ({self.current_x},{self.current_y})")
        
        dx = prev_x - self.current_x
        dy = prev_y - self.current_y
        
        if dx > 0:
            target_heading = Direction.EAST
        elif dx < 0:
            target_heading = Direction.WEST
        elif dy > 0:
            target_heading = Direction.NORTH
        elif dy < 0:
            target_heading = Direction.SOUTH
        else:
            target_heading = self.current_heading
        
        if self.move_to(prev_x, prev_y, target_heading):
            self.current_x = prev_x
            self.current_y = prev_y
            self.current_heading = target_heading
            return True
        return False

    def exploration_step(self):
        ray_buffer.clear()
        
        if isinstance(self.current_heading, int):
            self.current_heading = Direction(self.current_heading)
        
        print(f"\n{'='*60}")
        print(f"[STEP] Position: ({self.current_x}, {self.current_y}) Heading: {self.current_heading.name}")
        print(f"{'='*60}")
        
        color = self.check_tile_color()
        self.colors[self.current_x][self.current_y] = color
        
        if color == "black":
            print(f"[COLOR] BLACK tile detected - marking unreachable and backtracking")
            self.map[self.current_x][self.current_y] = TileState.BLACK
            if self.backtrack():
                print(f"[BACKTRACK] Successfully returned, continuing DFS")
                return
            else:
                print(f"[BACKTRACK] Failed to backtrack")
                self.exploration_complete = True
                return
        if color == "blue":
            print(f"[COLOR] BLUE tile detected - pausing 5 seconds")
            send_command(self.motor_controller.arduino, 'S')
            time.sleep(5)
            print(f"[COLOR] Resuming after blue tile pause")
        elif color == "red":
            print(f"[COLOR] RED tile detected - no action, continuing")
        elif color == "silver":
            print(f"[COLOR] SILVER checkpoint detected at ({self.current_x}, {self.current_y})")
        
        self.mark_current_tile_visited()
        self.scan_for_victims()
        
        detected_walls = self.get_walls()
        self.walls[self.current_x][self.current_y] = detected_walls
        
        unvisited_neighbors = self.get_unvisited_neighbors()
        
        if unvisited_neighbors:
            next_dir = self.get_next_neighbor(unvisited_neighbors)
            print(f"[CHOOSE] Next direction: {next_dir.name}")
            
            self.stack.append((self.current_x, self.current_y))
            
            dx, dy = self.get_direction_offset(next_dir)
            next_x, next_y = self.current_x + dx, self.current_y + dy
            
            result = self.move_to(next_x, next_y, next_dir)
            if result == "hole":
                self.stack.pop()
                print(f"[HOLE] Tile ({next_x},{next_y}) is a hole - marking unreachable")
                self._ensure_tile_exists(next_x, next_y)
                self.map[next_x][next_y] = TileState.BLACK
            elif result:
                self.current_x = next_x
                self.current_y = next_y
                self.current_heading = next_dir
            else:
                self.stack.pop()
                print("[ERROR] Move failed, staying at current tile")
        else:
            print("[DEAD END] No unvisited neighbors, backtracking")
            if not self.backtrack():
                self.exploration_complete = True

    def run_exploration(self, max_steps: int = 1000):
        print(f"\n[START] DFS Exploration")
        print(f"[START] Starting position: ({self.start_x}, {self.start_y}) facing NORTH")
        print(f"[START] Grid expands dynamically, supports negative coordinates")
        
        step = 0
        while not self.exploration_complete and step < max_steps:
            try:
                self.exploration_step()
                step += 1
                time.sleep(0.2)
            except KeyboardInterrupt:
                print("\n[INTERRUPT] Stopped by user")
                break
            except Exception as e:
                print(f"[ERROR] {e}")
                import traceback
                traceback.print_exc()
                break
        
        print(f"\n[END] Steps: {step} | Tiles visited: {self.total_tiles_visited}")
        print(f"[END] Final position: ({self.current_x}, {self.current_y})")
        return self.exploration_complete