import math


class Communication:
    # HOST = "127.0.0.1" # "172.16.165.216"
    HOST = "172.16.165.137"
    # HOST = "0.0.0.0"
    PORT = 30007


class RobotConfiguration:
    # DIST_THRESHOLD = 0.15
    DIST_THRESHOLD = 0.4
    ANGLE_THRESHOLD = math.pi / 360.0
    LIFT_VALUE = 0.004
    TWIST_MAX_COUNT = 5
    LIFT_PLATFORM_POSITION_MAX = 0.4
    LIFT_PLATFORM_POSITION_MIN = 0.0
    LIFT_THRESHOLD = 0.1
    


class FileConfiguration:
    MAP_FILE_PATH = "/home/ai/catkin_ws/src/husky/husky_behavior/maps/map_potenit.txt"