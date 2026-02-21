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
MOVE_STEPS = 10 # how many steps in the path the car should move at a time

ANGLES = np.arange(-45, 46, 5)
MIN_SCAN_DISTANCE = 4
MAX_SCAN_DISTANCE = 60

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
def inBounds(x, y):
    return 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE

# normalize angle to range [-pi, pi]
def normalizeAngle(angle):
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


def markObstacle(grid_x, grid_y, radius=10):
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

        if inBounds(grid_x, grid_y):
            markObstacle(grid_x, grid_y)


# VEHICLE MOTION MODEL

# ensures that we can only plan for motions that can be done by the picar
# returns position (x, y) and rotation (yaw) of car after moving STEP_CM
def simulateMotion(x, y, yaw, steer_deg):
    steer = math.radians(steer_deg)

    # straight motion case when steer is close to 0
    if abs(steer) < 1e-3:
        return (x + STEP_CM * math.cos(yaw), y + STEP_CM * math.sin(yaw), yaw)

    # turning motion
    R = WHEELBASE / math.tan(steer) # Turning radius
    dtheta = STEP_CM / R # Heading change

    nx = x + R * (math.sin(yaw + dtheta) - math.sin(yaw))
    ny = y - R * (math.cos(yaw + dtheta) - math.cos(yaw))

    return nx, ny, normalizeAngle(yaw + dtheta)


# checks if the simulated position collides with an obstacle in the grid
def isCollision(x, y):
    gx = int(origin_x + x)
    gy = int(origin_y + y)

    if not inBounds(gx, gy):
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
def reconstructPath(node):
    path = []
    while node:
        path.append((node.x, node.y))
        node = node.parent
    return path[::-1]


# Hybrid A* search
def hybridAstar(goal_local, local_start=(0, 0, 0)):
    start = Node(*local_start, cost=0)

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
            return reconstructPath(current)

        key = current.key()
        if key in visited:
            continue
        visited.add(key)

        # Expand using discrete steering angles
        for steer in STEER_SET:
            nx, ny, nyaw = simulateMotion(
                current.x, current.y, current.yaw, steer
            )

            if isCollision(nx, ny):
                continue

            cost = current.cost + STEP_CM
            node = Node(nx, ny, nyaw, cost, current)

            priority = cost + HEURISTIC_WEIGHT * heuristic(nx, ny, goal_local)
            heapq.heappush(open_list, (priority, next(counter), node))

    return None


# converts global goal into robot's local coordinates
def getLocalGoal(target_goal):
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

def executePath(path, steps_to_run=MOVE_STEPS):
    num_steps = min(steps_to_run, len(path) - 1)

    for i in range(1, num_steps + 1):
        x, y, yaw = robot_pose

        dx = path[i][0] - path[i - 1][0]
        dy = path[i][1] - path[i - 1][1]

        segment_yaw = math.atan2(dy, dx)
        yaw_error = normalizeAngle(segment_yaw - yaw)

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

        nx, ny, nyaw = simulateMotion(x, y, yaw, steer_angle)
        robot_pose[0] = nx
        robot_pose[1] = ny
        robot_pose[2] = nyaw


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
            if inBounds(gx, gy):
                path_set.add((gy, gx))

    goal_cell = None
    if goal is not None:
        gx = int(round(origin_x + robot_pose[0] + goal[0]))
        gy = int(round(origin_y + robot_pose[1] + goal[1]))
        if inBounds(gx, gy):
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
    local_start = (0, 0, 0)

    try:
        while True:
            print("Scanning")
            scanEnvironment()

            local_goal = getLocalGoal(target_goal)

            if math.hypot(*local_goal) < GOAL_TOLERANCE:
                print("GOAL REACHED!")
                break

            print("Planning")
            path = hybridAstar(local_goal, local_start=local_start)
            print(path)

            printGridTerminal(path, local_goal)

            if path:
                print("Executing")
                executePath(path, MOVE_STEPS)
            else:
                print("No path found")

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("Stopping car")
        px.set_cam_pan_angle(0)
        px.stop()


if __name__ == "__main__":
    main()