#                 DFS
#                 DFS DFS DFS DFS DFS DFS DFS
from movement import check_directions, move_one_tile, turn
from camera import check_victim

# class Node:
#     def __init__(self, state, parent):
#         self.state = state
#         self.parent = parent

# class RobotDFS:
#     def __init__(self, laser, scan, arduino, grid_size=5):
#         self.laser = laser
#         self.scan = scan
#         self.arduino = arduino
#         self.grid_size = grid_size
#         self.visited = set()
#         self.current_pos = (0, 0)
#         self.direction = 0
#         self.victims = []

#     def solve(self):
#         stack = [Node(self.current_pos, None)]

#         while stack:
#             node = stack.pop()

#             if node.state in self.visited:
#                 continue

#             self.visited.add(node.state)
#             self.current_pos = node.state
#             print(f"Exploring: {node.state}")

#             # Check for victim at current position
#             victim = check_victim("right")
#             if victim:
#                 print(f"Victim found at {node.state}: {victim}")
#                 self.victims.append((node.state, victim))

#             # Get neighbors
#             neighbors = self.get_valid_neighbors(node.state)

#             for next_state in neighbors:
#                 if next_state not in self.visited:
#                     stack.append(Node(next_state, node))
#                     # Move robot to next state
#                     self.move_to(next_state)

#         print(f"Exploration complete. Victims found: {self.victims}")
#         return self.victims

#     def get_valid_neighbors(self, state):
#         i, j = state
#         neighbors = []

#         # Check all 4 directions
#         directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

#         for di, dj in directions:
#             ni, nj = i + di, j + dj
#             # Check if within grid
#             if 0 <= ni < self.grid_size and 0 <= nj < self.grid_size:
#                 # Check if no obstacle
#                 r = self.laser.doProcessSimple(self.scan)
#                 if r:
#                     directions_check = check_directions(self.scan)
#                     # Map grid direction to robot direction
#                     if di == -1 and directions_check["front"]:  # Moving up   
#                         neighbors.append((ni, nj))
#                     elif di == 1 and directions_check["back"]:   # Moving down
#                         neighbors.append((ni, nj))
#                     elif dj == -1 and directions_check["left"]:  # Moving left
#                         neighbors.append((ni, nj))
#                     elif dj == 1 and directions_check["right"]:  # Moving right
#                         neighbors.append((ni, nj))

#         return neighbors

#     def move_to(self, next_state):
#         current_i, current_j = self.current_pos
#         next_i, next_j = next_state

#         # Calculate direction to move
#         if next_i < current_i:  # Move forward
#             move_one_tile(self.laser, self.scan, self.arduino)
#         elif next_i > current_i:  # Move backward
#             turn(self.laser, self.scan, self.arduino, "left")
#             turn(self.laser, self.scan, self.arduino, "left")
#             move_one_tile(self.laser, self.scan, self.arduino)
#         elif next_j < current_j:  # Move left
#             turn(self.laser, self.scan, self.arduino, "left")
#             move_one_tile(self.laser, self.scan, self.arduino)
#         elif next_j > current_j:  # Move right
#             turn(self.laser, self.scan, self.arduino, "right")
#             move_one_tile(self.laser, self.scan, self.arduino)