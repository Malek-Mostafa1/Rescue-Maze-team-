import time
import json
from collections import deque
from src.shared import ray_buffer
from src.arduino_handler import send_command
from src.dfs import TileState, Direction

class GreedyMazeExplorer:
    def __init__(self, width: int, height: int, start_x: int = 0, start_y: int = 0):
        self.width = width
        self.height = height
        self.start_x = start_x
        self.start_y = start_y
        
        # Dictionaries for dynamic grid (supports negative coordinates)
        self.map = {}
        self.walls = {}
        self.victims = {}
        self.colors = {}
        
        self._ensure_tile_exists(start_x, start_y)
        
        self.current_x = start_x
        self.current_y = start_y
        self.current_heading = Direction.NORTH
        
        # Checkpoint tracking
        self.last_checkpoint_x = start_x
        self.last_checkpoint_y = start_y
        
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
        
        wall_threshold = 27  # Slightly less than tile size for safety
        rays = self.lidar.get_rays()
        
        wall_data = {
            'N': (rays['N'] > 5) and (rays['N'] < wall_threshold),
            'E': (rays['E'] > 5) and (rays['E'] < wall_threshold),
            'S': (rays['S'] > 5) and (rays['S'] < wall_threshold),
            'W': (rays['W'] > 5) and (rays['W'] < wall_threshold),
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

    def get_open_neighbors(self, x, y):
        neighbors = []
        if x not in self.walls or y not in self.walls[x]: 
            return neighbors
            
        walls = self.walls[x][y]
        for direction in Direction:
            wall_key = self.get_wall_key_for_direction(direction)
            if not walls[wall_key]: # No wall blocking
                dx, dy = self.get_direction_offset(direction)
                nx, ny = x + dx, y + dy
                self._ensure_tile_exists(nx, ny)
                # Never traverse black tiles
                if self.map[nx][ny] != TileState.BLACK:
                    neighbors.append((nx, ny, direction))
        return neighbors

    def get_all_unvisited(self):
        """Scans the map dictionary and returns a list of all UNVISITED coordinates."""
        unvisited = []
        for x in self.map:
            for y in self.map[x]:
                if self.map[x][y] == TileState.UNVISITED:
                    unvisited.append((x, y))
        return unvisited

    def calculate_path_cost(self, path, starting_heading):
        """Calculates physical driving cost: 1 per tile + turn penalties."""
        if not path:
            return float('inf')
            
        cost = 0
        current_heading = starting_heading
        
        for nx, ny, target_dir in path:
            # Calculate turn penalty (0=Straight, 1=Left/Right, 2=U-Turn)
            turn_diff = (target_dir - current_heading) % 4
            turn_cost = min(turn_diff, 4 - turn_diff)
            
            # Add 1 for the forward movement, plus the turn penalty
            cost += (1 + turn_cost) 
            current_heading = target_dir
            
        return cost

    def _bfs_to_target(self, target_x, target_y):
        """Standard BFS that calculates the shortest grid path to a specific tile."""
        queue = deque()
        start = (self.current_x, self.current_y)
        queue.append((start, []))
        visited_search = set([start])
        while queue:
            (cx, cy), path = queue.popleft()
            if cx == target_x and cy == target_y:
                return path
            # Only traverse through VISITED tiles (or the starting tile itself)
            if self.map[cx][cy] == TileState.VISITED or (cx, cy) == start:
                for nx, ny, direction in self.get_open_neighbors(cx, cy):
                    if (nx, ny) not in visited_search:
                        visited_search.add((nx, ny))
                        queue.append(((nx, ny), path + [(nx, ny, direction)]))
        return None

    def find_path_to_nearest_unvisited(self):
        """Brute-forces the turn-weighted cost to every known unvisited tile and picks the best."""
        unvisited_tiles = self.get_all_unvisited()
        
        if not unvisited_tiles:
            return None
        best_path = None
        lowest_cost = float('inf')
        for tx, ty in unvisited_tiles:
            # Get the raw tile path
            path = self._bfs_to_target(tx, ty)
            
            if path:
                # Calculate what it actually costs the hardware to drive it
                cost = self.calculate_path_cost(path, self.current_heading)
                
                # Keep the cheapest one
                if cost < lowest_cost:
                    lowest_cost = cost
                    best_path = path
        return best_path

    def move_to(self, target_x: int, target_y: int, target_heading: Direction):
        if not self.motor_controller:
            return False
        
        # Calculate relative turn needed
        delta = (target_heading - self.current_heading) % 4
        
        if delta == 0:
            x = self.motor_controller.move_forward_one_tile()
            print("movetooooooooooooooooooo  ",x)
            return x
        elif delta == 1:
            if not self.motor_controller.turn_relative(1): return False
            return self.motor_controller.move_forward_one_tile()
        elif delta == 3:
            if not self.motor_controller.turn_relative(-1): return False
            return self.motor_controller.move_forward_one_tile()
        elif delta == 2:
            if not self.motor_controller.turn_relative(2): return False
            return self.motor_controller.move_forward_one_tile()
        return False

    def mark_current_tile_visited(self):
        self._ensure_tile_exists(self.current_x, self.current_y)
        self.map[self.current_x][self.current_y] = TileState.VISITED
        self.total_tiles_visited += 1
        print(f"[VISITED] ({self.current_x},{self.current_y}) total={self.total_tiles_visited}")

    def exploration_step(self):
        ray_buffer.clear()
    
        if isinstance(self.current_heading, int):
            self.current_heading = Direction(self.current_heading)
        
        print(f"\n{'='*60}")
        print(f"[STEP] Position: ({self.current_x}, {self.current_y}) Heading: {self.current_heading.name}")
        print(f"[STEP] Last checkpoint: ({self.last_checkpoint_x}, {self.last_checkpoint_y})")
        print(f"{'='*60}")
        
        # Check tile color FIRST (before marking visited)
        color = self.check_tile_color()
        self.colors[self.current_x][self.current_y] = color
        
        if color == "black":
            print(f"[COLOR] WARNING: Stationary black tile read. Ignoring to prevent death spin. Hardware will handle holes during transit.")
            # Do NOT step_back() here. If we are here, we are on a safe tile.
        elif color == "blue":
            print(f"[COLOR] BLUE tile detected - pausing 5 seconds")
            send_command(self.motor_controller.arduino, 'S')
            time.sleep(5)
            print(f"[COLOR] Resuming after blue tile pause")
        elif color == "red":
            print(f"[COLOR] RED tile detected - no action, continuing")
        elif color == "silver":
            print(f"[COLOR] SILVER checkpoint detected at ({self.current_x}, {self.current_y})")
            self.last_checkpoint_x = self.current_x
            self.last_checkpoint_y = self.current_y
            print(f"[CHECKPOINT] Updated last checkpoint to ({self.last_checkpoint_x}, {self.last_checkpoint_y})")
        
        # Mark current tile as visited
        self.mark_current_tile_visited()
        
        # Check for victims
        self.scan_for_victims()
        
        # Detect walls
        detected_walls = self.get_walls()
        self.walls[self.current_x][self.current_y] = detected_walls
        
        # Create neighbor tiles based on detected walls
        for direction in Direction:
            wall_key = self.get_wall_key_for_direction(direction)
            if not detected_walls[wall_key]:  # No wall blocking this direction
                dx, dy = self.get_direction_offset(direction)
                nx, ny = self.current_x + dx, self.current_y + dy
                self._ensure_tile_exists(nx, ny)  # Creates as UNVISITED by default

        # Calculate optimal path to nearest unvisited tile
        path = self.find_path_to_nearest_unvisited()
        
        if path:
            # path[0] holds the next immediate step to get towards the goal
            next_x, next_y, next_dir = path[0]
            print(f"[BFS] Next step along route: move {next_dir.name} to ({next_x}, {next_y})")
            
            # Execute step
            result = self.move_to(next_x, next_y, next_dir)
            
            if result == "hole":
                # Robot detected a hole mid-move and already backed up to current tile
                print(f"[HOLE] Tile ({next_x},{next_y}) is a hole - marking as BLACK, will not revisit")
                self._ensure_tile_exists(next_x, next_y)
                self.map[next_x][next_y] = TileState.BLACK
                # Position stays at current_x, current_y (robot backed up)
                # Heading updated to next_dir since it turned to attempt the move
                self.current_heading = next_dir
            elif result:
                self.current_x = next_x
                self.current_y = next_y
                self.current_heading = next_dir
            else:
                print("[ERROR] Move failed, staying at current tile")
        else:
            print("[BFS] No reachable unvisited tiles found. Maze is completely explored.")
            self.exploration_complete = True

    def run_exploration(self, max_steps: int = 1000):
        print(f"\n[START] Greedy BFS Exploration")
        print(f"[START] Starting position: ({self.start_x}, {self.start_y}) facing NORTH")
        
        step = 0
        while not self.exploration_complete and step < max_steps:
            try:
                self.exploration_step()
                step += 1
                time.sleep(0.2)  # Small delay between steps
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
        print(f"[END] Last checkpoint: ({self.last_checkpoint_x}, {self.last_checkpoint_y})")
        return self.exploration_complete