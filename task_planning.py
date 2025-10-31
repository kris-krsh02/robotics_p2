# Input format
# (start_point, end_point)
# ((1,2), (3, 4))
# end_point cannot be reached before the corresponding start point has been reached
import math
import pprint


class TaskPlanner:
    def __init__(self):
        self.tasks = {}  # store all start-end point combinations
        self.eligible_points = set()  # points that can be visited next
        self.distances = {}

    def parse_input(self, tasks_input: str):
        tasks_input = tasks_input.split("\n")
        for line in tasks_input:
            # Remove whitespace and delimiters
            line = line.strip()
            line = line.replace("(", "")
            line = line.replace(")", "")
            line = line.replace(",", "")

            # Split line into the coordinates
            if line:
                line = line.split(" ")
                coordinates = []
                for c in line:
                    if c.isnumeric():
                        coordinates.append(int(c))
                self.tasks[tuple(coordinates[0:2])] = tuple(coordinates[2:4])
                self.eligible_points.add(tuple(coordinates[0:2]))
            else:
                break

    def plan(self, curr_position: tuple):
        path = []
        while self.eligible_points:
            next_point = self.get_nearest_neighbor(curr_position)
            path.append(next_point)
            if next_point in self.tasks.keys():
                self.eligible_points.add(self.tasks[next_point])
            self.eligible_points.discard(next_point)

        return path

    def get_nearest_neighbor(self, p1: tuple):
        nearest_neighbor, nearest_distance = None, None

        for p2 in self.eligible_points:
            try:
                curr_distance = self.distances[(p1, p2)]
            except:
                curr_distance = self.distance(p1, p2)
            if nearest_distance is None or curr_distance < nearest_distance:
                nearest_neighbor, nearest_distance = p2, curr_distance

        return nearest_neighbor

    def distance(self, p1: tuple, p2: tuple):
        """
        Calculate distance between 2 points.

        Args:
            p1 (tuple): Point 1
            p2 (tuple): Point 2

        Returns:
            float: The distance between 2 points.
        """
        return math.sqrt(math.pow(p2[0] - p1[0], 2) + math.pow(p2[1] - p1[1], 2))

    def construct_distance_matrix(self):
        all_points = [p for p in self.tasks.values()]
        all_points.extend([p for p in self.tasks.keys()])
        for p1 in all_points:
            for p2 in all_points:
                if p1 == p2:
                    continue
                curr_distance = self.distance(p1, p2)
                self.distances[(p1, p2)] = curr_distance
        return self.distances


if __name__ == "__main__":
    TaskPlanner = TaskPlanner()
    TaskPlanner.parse_input("input.txt")
    print(TaskPlanner.plan((0, 0)))
    pprint.pprint(TaskPlanner.construct_distance_matrix())
