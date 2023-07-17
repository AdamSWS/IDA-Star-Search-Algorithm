import sys
import time
from heapq import heappush, heappop
# For assignment 5, please specify manhattan distance or misplaced tiles as the heuristic function


class Search:

  def goal_test(self, cur_tiles):
    return cur_tiles == [
      '1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13',
      '14', '15', '0'
    ]

  def manhattan_distance(self, tiles):
    # calculates the manhattan distance of the tiles on the board
    return sum(
      abs((ind / 4) - ((int(tile) - 1) / 4)) +
      abs((ind % 4) - ((int(tile) - 1) % 4)) for ind, tile in enumerate(tiles)
      if tile != '0')

  def misplaced_tiles(self, tiles):
    # counts the misplaced tiles on the board
    return sum(1 for i, tile in enumerate(tiles)
               if tile != '0' and tile != str(i + 1))

  def get_children(self, tiles):
    # gets the children states given the inputted tiles
    children = []
    i = tiles.index('0')
    x, y = divmod(i, 4)
    for dx, dy, direction in [(0, -1, 'L'), (0, 1, 'R'), (-1, 0, 'U'),
                              (1, 0, 'D')]:
      if 0 <= x + dx < 4 and 0 <= y + dy < 4:
        j = (x + dx) * 4 + (y + dy)
        new_tiles = tiles[:]
        new_tiles[i], new_tiles[j] = new_tiles[j], new_tiles[i]
        children.append((new_tiles, direction))
    return children

  def ida_star_manhattan_distance(self, initial_tiles):
    start_time = time.time()
    expanded_nodes = 0
    bound = self.manhattan_distance(initial_tiles)
    path = [initial_tiles]
    movements = []
    while True:
      t, expanded_nodes_ = self.search_manhattan_distance(path, 0, bound, movements, expanded_nodes)
      expanded_nodes += expanded_nodes_
      if t == "FOUND":
        end_time = time.time()
        time_taken = end_time - start_time
        memory_consumed = sys.getsizeof(path) + sys.getsizeof(movements)
        return movements, expanded_nodes, time_taken, memory_consumed
      if t == float('inf'):
        return None, expanded_nodes, None, None
      bound = t

  def search_manhattan_distance(self, path, g, bound, movements, expanded_nodes):
    node = path[-1]
    f = g + self.manhattan_distance(node)
    if f > bound:
      return f, expanded_nodes
    if self.goal_test(node):
      return "FOUND", expanded_nodes
    min_cost = float('inf')
    children = self.get_children(node)
    expanded_nodes += len(children)
    for child, move in children:
      if child not in path:
        path.append(child)
        if move:
          movements.append(move)
        t, expanded_nodes = self.search_manhattan_distance(path, g + 1, bound, movements, expanded_nodes)
        if t == "FOUND":
          return "FOUND", expanded_nodes
        if t < min_cost:
          min_cost = t
        path.pop()
        if move:
          movements.pop()
    return min_cost, expanded_nodes

  def ida_star_misplaced_tiles(self, initial_tiles):
    start_time = time.time()
    expanded_nodes = 0
    bound = self.misplaced_tiles(initial_tiles)
    path = [(initial_tiles, None)]
    movements = []
    while True:
      t, expanded_nodes_ = (self.search_misplaced_tiles(path, 0, bound, movements, expanded_nodes))
      expanded_nodes += expanded_nodes_
      if t == "FOUND":
        end_time = time.time()
        time_taken = end_time - start_time
        memory_consumed = sys.getsizeof(path) + sys.getsizeof(movements)
        return movements, expanded_nodes, time_taken, memory_consumed
      if t == float('inf'):
        return None, expanded_nodes, None, None
      bound = t

  def search_misplaced_tiles(self, path, g, bound, movements, expanded_nodes):
    node, action = path[-1]
    f = g + self.misplaced_tiles(node)
    if f > bound:
      return f, expanded_nodes
    if self.goal_test(node):
      return "FOUND", expanded_nodes
    min_cost = float('inf')
    children = self.get_children(node)
    expanded_nodes += len(children)
    for child, move in children:
      if child not in [n for n, a in path]:
        path.append((child, move))
        if move:
          movements.append(move)
        t, expanded_nodes = self.search_misplaced_tiles(path, g + 1, bound, movements, expanded_nodes)
        if t == "FOUND":
          return "FOUND", expanded_nodes
        if t < min_cost:
          min_cost = t
        path.pop()
        if move:
          movements.pop()
    return min_cost, expanded_nodes

  def solve(self, initial_state, heuristic="misplaced tiles"):
    initial_list = initial_state.split(" ")
    if heuristic == "manhattan":
      path, expanded_nodes, time_taken, memory_consumed = self.ida_star_manhattan_distance(initial_list)
    if heuristic == "misplaced tiles":
      path, expanded_nodes, time_taken, memory_consumed = self.ida_star_misplaced_tiles(initial_list)
    # print the program data
    print("Moves: " + " ".join(path))
    print("Number of expanded Nodes: " + str(expanded_nodes))
    print("Time Taken: " + str(time_taken))
    print("Max Memory (Bytes): " + str(memory_consumed))
    return "".join(path)


if __name__ == '__main__':
  agent = Search()
  initial_list = "5 1 2 3 9 6 7 4 13 10 11 8 0 14 15 12"
  solution = agent.solve(initial_list)
  if solution is None:
    print("No solution found.")