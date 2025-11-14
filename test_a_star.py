import unittest
from a_star import astar, get_heuristic, get_distance, Node

class TestAStar(unittest.TestCase):

    def test_get_heuristic(self):
        self.assertAlmostEqual(get_heuristic((0, 0), (3, 4)), 5.0)
        self.assertAlmostEqual(get_heuristic((0, 0), (0, 0)), 0.0)

    def test_get_distance(self):
        self.assertAlmostEqual(get_distance((0, 0), (3, 4)), 5.0)
        self.assertAlmostEqual(get_distance((0, 0), (0, 0)), 0.0)
    
    # Test graceful failure on invalid inputs
    def test_astar_start_out_of_bounds(self):
        map_ = [[0]]
        with self.assertRaises(AssertionError):
            astar((1, 0), (0, 0), map_)

    def test_astar_goal_out_of_bounds(self):
        map_ = [[0]]
        with self.assertRaises(AssertionError):
            astar((0, 0), (1, 0), map_)

    def test_astar_empty_map(self):
        with self.assertRaises(AssertionError):
            astar((0, 0), (0, 0), [])

    # Functional tests
    def test_astar_trivial_path(self):
        map_ = [
            [0, 0, 0],
            [0, 0, 0],
            [0, 0, 0]
        ]
        start = (0, 0)
        end = (2, 2)
        path = astar(start, end, map_)
        
        # To verify the path, we need to reconstruct it from the returned node
        result_path = []
        current = path
        while current:
            result_path.append(current.position)
            current = current.parent
        result_path.reverse()

        # Verify that a path was found
        self.assertIsNotNone(path)

        # Verify start and end positions
        self.assertEqual(result_path[0], start)
        self.assertEqual(result_path[-1], end)

        # Verify shaortest path
        expected_path = [(0, 0), (1, 1), (2, 2)]
        for i in range(len(expected_path)):
            self.assertEqual(expected_path[i], result_path[i])


    def test_astar_no_path(self):
        map_ = [
            [0, 1, 0],
            [0, 1, 0],
            [0, 1, 0]
        ]
        start = (0, 0)
        end = (0, 2)
        path = astar(start, end, map_)

        # Verify that no path was found
        self.assertIsNone(path)

    def test_astar_bigger_map_path(self):
        map_ = [
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 1, 1, 1, 0],
            [0, 1, 0, 0, 0, 0, 1, 0],
            [0, 0, 0, 1, 1, 0, 0, 0],
            [0, 1, 0, 0, 0, 0, 1, 0],
            [0, 1, 1, 1, 1, 0, 1, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
        ]
        start = (0, 0)
        end = (8, 7)
        path = astar(start, end, map_)

        # Verify that a path was found
        self.assertIsNotNone(path)
        
        result_path = []
        current = path
        while current:
            result_path.append(current.position)
            current = current.parent
        result_path.reverse()

        # Verify start and end positions        
        self.assertEqual(result_path[0], start)
        self.assertEqual(result_path[-1], end)

        # Verify shaortest path
        expected_path = [(0, 0), (0, 1), (0, 2), (1, 3), (2, 4), (3, 5), (3, 6), (4, 7), (5, 7), (6, 7), (7, 7), (8, 7)]
        for i in range(len(expected_path)):
            self.assertEqual(expected_path[i], result_path[i])

    def test_astar_bigger_map_no_path(self):
        map_ = [
            [0, 0, 0, 0, 0],
            [0, 1, 1, 1, 0],
            [0, 1, 0, 1, 0],
            [0, 1, 1, 1, 0],
            [0, 0, 0, 0, 0]
        ]
        start = (0, 0)
        end = (2, 2)
        path = astar(start, end, map_)

        # Verify that no path was found
        self.assertIsNone(path)

if __name__ == '__main__':
    unittest.main()
