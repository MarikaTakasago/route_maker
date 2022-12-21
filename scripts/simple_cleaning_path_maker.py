#!/usr/bin/env python3

import rospy
import yaml
from geometry_msgs.msg import PoseStamped，Point
from nav_msgs.msg import Path, OccupancyGrid

from route_maker.msg import Area, AreaArray, PathArray

from visualization_msgs.msg import Marker, MarkerArray

class SimpleCleaningPathMaker:
    def __init__(self,use_yaml):
        rospy.init_node('simple_cleaning_path_maker')
        print('=== simple_cleaning_path_maker ===')

        self.use_yaml = use_yaml
        self.get_map = False
        self.get_area = False
        self.get_wall = False

        ## check param
        if use_yaml:
            if rospy.has_param('~AREA_YAML_FILE'):
                self.area_yaml_file = rospy.get_param('~AREA_YAML_FILE')
            if rospy.has_param('~WALL_YAML_FILE'):
                self.wall_yaml_file = rospy.get_param('~WALL_YAML_FILE')
            ## load yaml file
            self.area_yaml = self.load_area()
            self.wall_yaml = self.load_wall()

        if rospy.has_param('~DIST_FROM_WALL'):
            self.dist_from_wall = rospy.get_param('~DIST_FROM_WALL')
        if rospy.has_param('~BRUSH_WIDTH'):
            self.brush_width = rospy.get_param('~BRUSH_WIDTH')
        if rospy.has_param('~BRUSH_LENGTH'):
            self.brush_length = rospy.get_param('~BRUSH_LENGTH')

        self.global_map = OccupancyGrid()
        self.input_area = AreaArray()
        self.2d_map = [][]
        self.map_width = 0
        self.map_height = 0

        ## subscriber
        self.sub_global_map = rospy.Subscriber('/map', OccupancyGrid, self.global_map_callback, queue_size=1)
        self.sub_area = rospy.Subscriber('/area', AreaArray, self.area_callback, queue_size=1)

        ## publisher
        self.pub_path = rospy.Publisher('/path', Path, queue_size=1)

        def global_map_callback(self, msg):
            self.global_map = msg
            self.map_width = msg.info.width
            self.map_height = msg.info.height
            for i in range(self.map_height):
                for j in range(self.map_width):
                    self.2d_map[i][j] = msg.data[i*self.map_width+j]

            self.get_map = True

        def area_callback(self, msg):
            if not self.use_yaml:
                self.input_area = msg
                self.get_area = True

        def load_area(self):
            with open(self.area_yaml_file) as f:
                area_yaml = yaml.safe_load(f)
            self.get_area = True
            return area_yaml

        def load_wall(self):
            with open(self.wall_yaml_file) as f:
                wall_yaml = yaml.safe_load(f)
            self.get_wall = True
            return wall_yaml

        def make_path(self):
            ## make path
            ## start from start point
            ## first move along the wall to p2, p3, p4, then back to p1

