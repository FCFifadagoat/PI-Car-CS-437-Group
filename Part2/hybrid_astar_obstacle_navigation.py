import numpy as np
import time
import math
import heapq
import itertools
from picarx import Picarx

px = Picarx()

# Unique counter used to break ties in priority queue (Hybrid A*)
counter = itertools.count()

# MAP & SCANNING PARAMETERS

MAP_SIZE = 200
STEP_CM = 5 # Distance (cm) moved per planning step

ANGLES = np.arange(-45, 46, 5)
MIN_SCAN_DISTANCE = 8
MAX_SCAN_DISTANCE = 80

# VEHICLE PARAMETERS

WHEELBASE = 9.75 # Distance between front and rear axle (cm)
MAX_STEER = 30 # Maximum steering angle allowed (degrees)
STEER_SET = [-30, 0, 30] # Steering angles used during Hybrid A* expansion

FORWARD_POWER = 35
CM_PER_SEC = 25.5 # car moves roughly 25.5cm/s at 35 power

# PLANNING PARAMETERS

MAX_RANGE = 180 # Max search distance in planning
GOAL_TOLERANCE = STEP_CM * 2 # How close is considered "goal reached"
MAX_ITERATIONS = 15000 # Planner iteration cap (safety timeout)
HEURISTIC_WEIGHT = 1.2 # A* heuristic inflation factor

# GLOBAL STATE

# Occupancy grid (0 = free, 1 = obstacle)
grid = np.zeros((MAP_SIZE, MAP_SIZE))

# Robot origin in grid coordinates
origin_x = 30 # start at 30 so we can use the other 170 for pathing
origin_y = MAP_SIZE // 2

# Robot pose in global world frame: [x, y, yaw]
robot_pose = [0.0, 0.0, 0.0]

# UTILITY FUNCTIONS

# checks if provided x, y coordinate is inside the map
def in_bounds(x, y):
    return 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE

# normalize angle to range [-pi, pi]
def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

# Euclidean distance heuristic for A* algorithm
def heuristic(x, y, goal):
    return math.hypot(goal[0] - x, goal[1] - y)


# ULTRASONIC PROCESSING

def getDistance(samples=3):
    readings = [px.ultrasonic.read() for _ in range(samples)]
    readings = [d for d in readings if d > 0]

    if len(readings) < samples:
        return -1

    if np.std(readings) > 10:
        return -1

    return float(np.median(readings))


def markObstacle(grid_x, grid_y, radius=5):
    x_min = max(0, grid_x - radius)
    x_max = min(MAP_SIZE, grid_x + radius)
    y_min = max(0, grid_y - radius)
    y_max = min(MAP_SIZE, grid_y + radius)

    grid[y_min:y_max, x_min:x_max] = 1


def scanEnvironment():
    grid.fill(0)  # clear previous scan each time we scan the environment so we can always have an updated local grid map

    for angle in ANGLES:
        px.set_cam_pan_angle(angle)
        time.sleep(0.08)

        distance = getDistance()

        if distance == -1 or not (MIN_SCAN_DISTANCE <= distance <= MAX_SCAN_DISTANCE):
            continue

        angle_rad = math.radians(angle)

        # Convert polar reading to local Cartesian coordinates
        # 16 is the distance between the rear axle and the front of the ultrasonic sensor
        obj_x = 16 + distance * math.cos(angle_rad)
        obj_y = distance * math.sin(angle_rad)

        # Convert to grid coordinates
        grid_x = int(origin_x + obj_x)
        grid_y = int(origin_y + obj_y)

        if in_bounds(grid_x, grid_y):
            markObstacle(grid_x, grid_y)


# VEHICLE MOTION MODEL

# ensures that we can only plan for motions that can be done by the picar
# returns position (x, y) and rotation (yaw) of car after moving STEP_CM
def simulate_motion(x, y, yaw, steer_deg):
    steer = math.radians(steer_deg)

    # straight motion case when steer is close to 0
    if abs(steer) < 1e-3:
        return (x + STEP_CM * math.cos(yaw), y + STEP_CM * math.sin(yaw), yaw)

    # turning motion
    R = WHEELBASE / math.tan(steer) # Turning radius
    dtheta = STEP_CM / R # Heading change

    nx = x + R * (math.sin(yaw + dtheta) - math.sin(yaw))
    ny = y - R * (math.cos(yaw + dtheta) - math.cos(yaw))

    return nx, ny, normalize_angle(yaw + dtheta)


# checks if the simulated position collides with an obstacle in the grid
def is_collision(x, y):
    gx = int(origin_x + x)
    gy = int(origin_y + y)

    if not in_bounds(gx, gy):
        return True # treat out-of-bounds as collision

    return grid[gy, gx] == 1


# HYBRID A* IMPLEMENTATION

# Node for Hybrid A* search. stores the
class Node:
    __slots__ = ("x", "y", "yaw", "cost", "parent")

    def __init__(self, x, y, yaw, cost, parent=None):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.cost = cost
        self.parent = parent

    # Reduces continuous state to grid cell + yaw.
    def key(self):
        return (
            int(self.x // STEP_CM),
            int(self.y // STEP_CM),
            int(self.yaw / math.radians(15)),
        )


# Reconstruct full path from goal node back to start
def reconstruct_path(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]


# Hybrid A* search
def hybrid_astar(goal_local):
    start = Node(0, 0, 0, 0)

    open_list = []
    heapq.heappush(open_list, (0, next(counter), start))
    visited = set()

    iterations = 0

    while open_list:
        iterations += 1

        if iterations > MAX_ITERATIONS:
            print("Hybrid A* timeout")
            return None

        _, _, current = heapq.heappop(open_list)

        # Skip nodes too far away
        if abs(current.x) > MAX_RANGE or abs(current.y) > MAX_RANGE:
            continue

        # Goal check
        if heuristic(current.x, current.y, goal_local) < GOAL_TOLERANCE:
            return reconstruct_path(current)

        key = current.key()
        if key in visited:
            continue
        visited.add(key)

        # Expand using discrete steering angles
        for steer in STEER_SET:
            nx, ny, nyaw = simulate_motion(
                current.x, current.y, current.yaw, steer
            )

            if is_collision(nx, ny):
                continue

            cost = current.cost + STEP_CM
            node = Node(nx, ny, nyaw, cost, current)

            priority = cost + HEURISTIC_WEIGHT * heuristic(nx, ny, goal_local)
            heapq.heappush(open_list, (priority, next(counter), node))

    return None


# converts global goal into robot's local coordinates
def get_local_goal(target_goal):
    x, y, yaw = robot_pose

    dx = target_goal[0] - x
    dy = target_goal[1] - y

    cos_yaw = math.cos(-yaw)
    sin_yaw = math.sin(-yaw)

    return (
        dx * cos_yaw - dy * sin_yaw,
        dx * sin_yaw + dy * cos_yaw,
    )


# executes a limited number of path segments

def execute_path(path, steps_to_run=None):
    steps_executed = 0
    total_segments = len(path) - 1

    if total_segments <= 0:
        return 0

    if steps_to_run is None:
        steps_to_run = total_segments

    segments_to_execute = min(steps_to_run, total_segments)

    for i in range(1, segments_to_execute + 1):
        x, y, yaw = robot_pose

        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]

        segment_yaw = math.atan2(dy, dx)
        yaw_error = normalize_angle(segment_yaw - yaw)

        # Convert heading error to steering angle
        steer_angle = np.clip(
            math.degrees(math.atan2(WHEELBASE * yaw_error, STEP_CM)),
            -MAX_STEER,
            MAX_STEER,
        )

        px.set_dir_servo_angle(steer_angle)

        move_time = STEP_CM / CM_PER_SEC
        px.forward(FORWARD_POWER)
        time.sleep(move_time)
        px.stop()

        # Update global pose estimate
        nx, ny, nyaw = simulate_motion(0, 0, yaw, steer_angle)
        robot_pose[0] += nx
        robot_pose[1] += ny
        robot_pose[2] = nyaw

        steps_executed += 1

    return steps_executed


def printGridTerminal(path=None, goal=None):
    car_length = 22
    car_width = 14

    half_length = car_length // 2
    half_width = car_width // 2

    path_set = set()
    if path:
        for px_, py_ in path:
            gx = int(round(origin_x + px_))
            gy = int(round(origin_y + py_))
            if in_bounds(gx, gy):
                path_set.add((gy, gx))

    goal_cell = None
    if goal is not None:
        gx = int(round(origin_x + goal[0]))
        gy = int(round(origin_y + goal[1]))
        if in_bounds(gx, gy):
            goal_cell = (gy, gx)

    for row in range(MAP_SIZE):
        row_str = ""
        for col in range(MAP_SIZE):

            # car footprint at origin
            if (origin_x - half_length <= col <= origin_x + half_length and
                origin_y - half_width <= row <= origin_y + half_width):
                row_str += "C"

            elif goal_cell and (row, col) == goal_cell:
                row_str += "G"

            elif (row, col) in path_set:
                row_str += "."

            else:
                row_str += str(int(grid[row, col]))

        print(row_str)


def main():
    target_goal = (125, 50)

    try:
        while True:
            print("Scanning")
            scanEnvironment()

            local_goal = get_local_goal(target_goal)

            if math.hypot(*local_goal) < GOAL_TOLERANCE:
                print("GOAL REACHED!")
                break

            print("Planning")
            path = hybrid_astar(local_goal)
            print(path)

            printGridTerminal(path, local_goal)

            if path:
                print("Executing 5 steps")
                execute_path(path, 5)
            else:
                print("No path found")

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("Stopping car")
        px.set_cam_pan_angle(0)
        px.stop()


if __name__ == "__main__":
    main()