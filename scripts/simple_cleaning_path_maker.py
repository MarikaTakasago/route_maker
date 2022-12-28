#!/usr/bin/env python3

import rospy
import yaml
from geometry_msgs.msg import PoseStamped，Point
from nav_msgs.msg import Path, OccupancyGrid

from route_maker.msg import Area, AreaArray, Wall, WallArray, PathArray

from visualization_msgs.msg import Marker, MarkerArray

class SimpleCleaningPathMaker:
    def __init__(self,use_yaml):
        rospy.init_node('simple_cleaning_path_maker')
        print('=== simple_cleaning_path_maker ===')

        self.use_yaml = use_yaml
        self.get_map = False
        self.get_area = False
        self.get_wall = False
        self.input_area = AreaArray()
        self.input_wall = WallArray()
        self.path = Path()

        self.global_map = OccupancyGrid()
        self.2d_map = [][]
        self.map_width = 0
        self.map_height = 0

        ## check param
        if use_yaml:
            if rospy.has_param('~AREA_YAML_FILE'):
                self.area_yaml_file = rospy.get_param('~AREA_YAML_FILE')
            if rospy.has_param('~WALL_YAML_FILE'):
                self.wall_yaml_file = rospy.get_param('~WALL_YAML_FILE')
            ## load yaml file
            self.area_yaml = self.load_area()
            self.wall_yaml = self.load_wall()

            ## make array
            self.make_area_array(self.input_area)
            self.make_wall_array(self.input_wall)

        if rospy.has_param('~HZ'):
            self.hz = rospy.get_param('~HZ')
        if rospy.has_param('~DIST_FROM_WALL'):
            self.dist_from_wall = rospy.get_param('~DIST_FROM_WALL')
        if rospy.has_param('~BRUSH_WIDTH'):
            self.brush_width = rospy.get_param('~BRUSH_WIDTH')
        if rospy.has_param('~BRUSH_LENGTH'):
            self.brush_length = rospy.get_param('~BRUSH_LENGTH')
        if rospy.has_param('~LINE_SPACE'):
            self.line_space = rospy.get_param('~LINE_SPACE')

        ## subscriber
        self.sub_global_map = rospy.Subscriber('/map', OccupancyGrid, self.global_map_callback, queue_size=1)
        self.sub_area = rospy.Subscriber('/area', AreaArray, self.area_callback, queue_size=1)
        self.sub_wall = rospy.Subscriber('/wall', WallArray, self.wall_callback, queue_size=1)

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

    def wall_callback(self, msg):
        if not self.use_yaml:
            self.input_wall = msg
            self.get_wall = True

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

    def make_area_array(self,area_array):
        for i in self.area_yaml['AREA']:
            area_array.areas.append(i)

    def make_wall_array(self,wall_array):
        for i in self.wall_yaml['WALL']:
            wall_array.walls.append(i)

    def make_path(self,wall_array,area):
        mini_path = Path()
        wall_in_area = WallArray()
        ## search wall in the area
        self.search_wall(wall_array,area,wall_in_area)
        start = area.start
        ## make path
        ## start from start point
        ## first move to the wall start point
        self.make_mini_path(mini_math,wall,area)
        ## add mini path to path
        self.path.poses.append(mini_path.poses)
        ## move along the wall

    def search_wall(self,wall_array,area,wall_in_area):
        ## search wall in the area
        x_min = min(area.p1.x,area.p3.x)
        x_max = max(area.p1.x,area.p3.x)
        y_min = min(area.p1.y,area.p3.y)
        y_max = max(area.p1.y,area.p3.y)
        for i in range(len(wall_array.walls)):
            wall_x_min = min(wall_array.walls[i].start.x,wall_array.walls[i].end.x)
            wall_x_max = max(wall_array.walls[i].start.x,wall_array.walls[i].end.x)
            wall_y_min = min(wall_array.walls[i].start.y,wall_array.walls[i].end.y)
            wall_y_max = max(wall_array.walls[i].start.y,wall_array.walls[i].end.y)
            if x_min <= wall_x_min and wall_x_max <= x_max and y_min <= wall_y_min and wall_y_max <= y_max:
                wall_in_area.walls.append(wall_array.walls[i])


    def main(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            if self.get_map and self.get_area and self.get_wall:
                ## update arrays
                if self.use_yaml:
                    new_area_array = AreaArray()
                    self.area_yaml = self.load_area()
                    self.make_area_array(new_area_array)
                    if new_area_array != self.input_area:
                        self.input_area = new_area_array
                        print ('update area array')

                    new_wall_array = WallArray()
                    self.wall_yaml = self.load_wall()
                    self.make_wall_array(new_wall_array)
                    if new_wall_array != self.input_wall:
                        self.input_wall = new_wall_array
                        print ('update wall array')

                ## make path
                for i in range(len(self.input_area.areas)):
                    self.make_path(self.input_wall.walls[i],self.input_area.areas[i])
                self.pub_path.publish(self.path)

        rate.sleep()

if __name__ == '__main__':
    simple_cleaning_path_maker = SimpleCleaningPathMaker()
    simple_cleaning_path_maker.main()

