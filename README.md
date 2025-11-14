# A* Pathfinding Algorithm

This project provides a Python implementation of the A* pathfinding algorithm. The algorithm finds the shortest path between a start and a goal point on a 2D grid, avoiding obstacles.

## Assumptions

1.  **Grid Representation**: The map is represented as a 2D list of integers. A value of `0` indicates a traversable cell, while any non-zero value (e.g., `1`) is considered an obstacle.
2.  **Movement**: The algorithm allows for 8-directional movement (horizontal, vertical, and diagonal).
3.  **Cost Function (g-cost)**: The cost of moving from one node to an adjacent one is calculated using the Euclidean distance.
4.  **Heuristic Function (h-cost)**: The heuristic estimate for the cost to reach the goal from a given node is also the Euclidean distance. This heuristic is admissible because it never overestimates the actual cost, ensuring that A* will find the shortest path.

## Algorithm Explanation

The A* algorithm finds the shortest path by prioritizing nodes that are likely to be on the best path. It does this by maintaining an "open list" of nodes to be evaluated, sorted by their `f-cost`, where `f = g + h`.

-   `g-cost`: The actual cost of the path from the start node to the current node.
-   `h-cost`: The estimated (heuristic) cost from the current node to the goal node.
-   `f-cost`: The total estimated cost of the path through the current node (`f = g + h`).

### Data Structures

-   **Open List**: A min-heap is used for the open list to efficiently retrieve the node with the lowest `f-cost`. This is crucial for the performance of the A* algorithm.
-   **Closed List**: A set is used for the closed list to store nodes that have already been evaluated. Sets provide fast lookups, insertions, and deletions.

### Pseudocode

```
function astar(start, goal, map):
    // Initialize data structures
    open_list = min-heap()
    closed_list = set()

    // Create start and goal nodes
    start_node = Node(position=start)
    goal_node = Node(position=goal)

    // Add the start node to the open list
    add start_node to open_list

    while open_list is not empty:
        // Get the node with the lowest f-cost
        current_node = get node with minimum f from open_list

        // If node has already been visited, skip
        if current_node in closed_list:
            continue

        // If goal is reached, reconstruct and return the path
        if current_node is goal_node:
            return reconstruct_path(current_node)

        // Add the current node to the closed list
        add current_node to closed_list

        // Explore neighbors
        for each neighbor of current_node:
            // Skip if neighbor is an obstacle or out of bounds
            if neighbor is not traversable or out of bounds:
                continue

            // Create a successor node
            successor_node = Node(position=neighbor_position, parent=current_node)

            // Skip if the successor is already in the closed list
            if successor_node in closed_list:
                continue

            // Calculate costs for the successor
            successor_node.g = current_node.g + distance(current_node, successor_node)
            successor_node.h = heuristic(successor_node, goal_node)
            successor_node.f = successor_node.g + successor_node.h

            // If a node with the same position as successor is in the open_list
            // with a lower f-cost, skip adding the successor
            if a better or equal node exists in open_list:
                continue

            // Add the new node to the open list
            add successor_node to open_list

    // If the open list becomes empty, no path was found
    return None
```

## How to Run

### Prerequisites

-   Python 3.x

### Running the Example

To run the example provided in `a_star.py`, which finds a path on a sample map and prints the result, execute the following command:

```bash
python3 a_star.py
```

### Running Tests

The project includes a test suite to verify the correctness of the A* implementation. To run the tests, use the following command:

```bash
python3 -m unittest test_a_star.py
```

### Demo Video
[peppermint_demo.webm](https://github.com/user-attachments/assets/5582045c-5651-428d-a31a-463dd671c4e7)
