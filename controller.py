from task_planning import TaskPlanner
from path_planning import PathPlanner
from navigation import Navigator
from avoid_obstacles import ObstacleAvoider
from position_tracker import PositionTracker


class Controller:
    def __init__(self):
        self.task_order = None


if __name__ == "__main__":
    controller = Controller()
    task_planner = TaskPlanner()
    path_planner = PathPlanner()
    navigator = Navigator()
    obstacle_avoider = ObstacleAvoider()
    position_tracker = PositionTracker()

    while True:
        print("Enter tasks to be completed by the robot:")
        lines = []
        line = input()

        while line and line.strip() != "exit":
            lines.append(line)
            line = input()

        if line == "exit":
            print("Stopping application ...")
            break

        task_input = "\n".join(lines)
        task_planner.parse_input(task_input)
        controller.task_order = task_planner.plan(
            (0, 0)
        )  # need to pass starting position

        print("Planned Task Order: " + str(controller.task_order))
        path_planner.set_angular_speed(
            navigator.angular_velocity
        )  # set angular speed from navigator

        # TODO: Assumed starting at (0,0,0), take in starting position as input later
        curr_pos = (0, 0, 0)
        for point in controller.task_order:
            time_to_turn = path_planner.get_vel_commands(curr_pos, point)
            navigator.run(obstacle_avoider, position_tracker, time_to_turn, point)

        print("All tasks completed. Enter new tasks or type 'exit' to quit.")
