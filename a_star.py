import heapq
from typing import List, Tuple

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start to current node
        self.h = 0  # Heuristic cost from current node to goal
        self.f = 0  # Total cost

    # Very important for node comparison in lists
    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f
    
    def __hash__(self):
        return hash(self.position)


def get_heuristic(position: Tuple[int, int], goal_position: Tuple[int, int]) -> int:
    """
    Calculate the Euclidean distance between the current position and the goal position for heuristic estimation.
    
    Args:
        position (Tuple[int, int]): Current position (x, y).
        goal_position (Tuple[int, int]): Goal position (x, y).
    
    Returns:
        float: Euclidean distance between position and goal_position.
    """
    return ((position[0] - goal_position[0]) ** 2 + (position[1] - goal_position[1]) ** 2) ** 0.5

def get_distance(source: Tuple[int, int], destination: Tuple[int, int]) -> int:
    """
    Calculate the Euclidean distance between the current position and the goal position for cost estimation.

    Args:
        source (Tuple[int, int]): Source position (x, y).
        destination (Tuple[int, int]): Destination position (x, y).
    
    Returns:
        float: Euclidean distance between source and destination.
    """
    return ((source[0] - destination[0]) ** 2 + (source[1] - destination[1]) ** 2) ** 0.5

def astar(start_point: Tuple[int, int], goal_point: Tuple[int, int], map: List[List[int]]) -> List[Tuple[int, int]]:
    """
    Implements the A* pathplanning algorithm using minheap.

    Args:
        start_point (Tuple[int, int]): Starting coordinates (x, y).
        goal_point (Tuple[int, int]): Goal coordinates (x, y).
        map (List[List[int]]): 2D grid map where 0 represents free cells and 1 represents obstacles.
    Returns:
        List[Tuple[int, int]] or None: The path from start to goal as a list of coordinates, or None if no path found.
    
    Raises:
        AssertionError: If start or goal points are out of map bounds or if the map is empty.
    
    """

    assert len(map) > 0 and len(map[0]) > 0, "Map must be non-empty"
    
    x_start, y_start = start_point
    assert 0 <= x_start < len(map) and 0 <= y_start < len(map[0]), "Start point out of bounds"

    x_goal, y_goal = goal_point
    assert 0 <= x_goal < len(map) and 0 <= y_goal < len(map[0]), "Goal point out of bounds"

    # Initialization
    start_node = Node(start_point)
    goal_node = Node(goal_point)

    open_list = []
    closed_list = set()

    heapq.heappush(open_list, start_node)

    while len(open_list) > 0:
        current_node = heapq.heappop(open_list)

        if current_node in closed_list:
            continue

        if current_node.position == goal_node.position:
            return current_node

        closed_list.add(current_node)

        # Assuming 8 possible movements (including diagonals)
        neighbours = [
            (0, -1), # Up
            (1, -1), # Up-Right
            (1, 0), # Right
            (1, 1), # Down-Right
            (0, 1), # Down
            (-1, 1), # Down-Left
            (-1, 0), # Left
            (-1, -1) # Up-Left
        ]

        for neighbour in neighbours:
            new_position = current_node.position[0] + neighbour[0], current_node.position[1] + neighbour[1]

            # Check if within bounds
            if not (0 <= new_position[0] < len(map) and 0 <= new_position[1] < len(map[0])):
                continue

            # Check if free cell
            if map[new_position[0]][new_position[1]] != 0:
                continue

            successor_node = Node(new_position, current_node)

            # If already evaluated, skip
            if successor_node in closed_list:
                continue
            
            # Update Heuristics
            successor_node.g = current_node.g + get_distance(current_node.position, successor_node.position)
            successor_node.h = get_heuristic(successor_node.position, goal_node.position)
            successor_node.f = successor_node.g + successor_node.h

            # If already in open list with a lower cost, skip
            if successor_node in open_list:
                existing_node = open_list[open_list.index(successor_node)]
                if successor_node.f >= existing_node.f:
                    continue

            heapq.heappush(open_list, successor_node)

    return None

def main():
    map = [
        [0, 0, 0, 0, 0],
        [0, 0, 1, 1, 0],
        [0, 1, 0, 0, 0],
        [0, 1, 1, 0, 0],
        [0, 0, 0, 0, 0]
    ]
    start = (0, 0)
    end = (4, 4)

    path = astar(start, end, map)
    if path:
        current = path
        result = []
        while current:
            result.append(current.position)
            current = current.parent
        result.reverse()
        print("Path found:", result)
    else:
        print("No path found")


if __name__ == '__main__':
    main()
