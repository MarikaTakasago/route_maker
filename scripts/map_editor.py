#!/usr/bin/env python3

## map edit flow ##
# 1. load map
# 2. get cleaning area from yaml
# 3. edit map with cleaning area
# 4. align line using hough transform
# 5. save map

import rospy
import yaml
import math
import cv2
import numpy as np
import copy

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, MapMetaData

from route_maker.msg import Area, AreaArray, Wall, WallArray

class MapEditor:
    def __init__(self):
        rospy.init_node('map_editor')
        print('=== Map Editor ===')

        ## check param
        if rospy.has_param('~map_file'):
            self.map_path = rospy.get_param('~map_file')
            print('map: ', self.map_path)
        if rospy.has_param('~area_file'):
            self.area_path = rospy.get_param('~area_file')
            print('area :' , self.area_path)
        if rospy.has_param('~wall_file'):
            self.wall_path = rospy.get_param('~wall_file')
            print('wall :' , self.wall_path)

        self.use_area = rospy.get_param('~use_area', False)
        self.use_wall = rospy.get_param('~use_wall', False)

        ## set param
        self.get_map = False
        self.get_area = False
        self.get_wall = False

        self.map = OccupancyGrid()
        self.areas = AreaArray()
        self.walls = WallArray()

        # make array
        if(self.use_area):
            self.area_yaml = self.load_area()
            self.make_area_array(self.areas)
        if(self.use_wall):
            self.wall_yaml = self.load_wall()
            self.walls = self.make_wall_array()
            # self.make_wall_array(self.walls)

        ## subscriber
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        ## publisher
        self.map_pub = rospy.Publisher('/edited_map', OccupancyGrid, queue_size=1)

    def map_callback(self, msg):
        self.map = msg
        print('map loaded')
        self.get_map = True
        print('map: ', self.map.info)

    def load_area(self):
        with open(self.area_path) as f:
            area_yaml = yaml.safe_load(f)
        self.get_area = True
        return area_yaml

    def load_wall(self):
        with open(self.wall_path) as f:
            wall_yaml = yaml.safe_load(f)
        self.get_wall = True
        return wall_yaml

    def make_area_array(self,area_array):
        areaa = Area()
        for area in self.area_yaml['AREA']:
            areaa.id = area['id']
            areaa.p1.x = area['p1']['x']
            areaa.p1.y = area['p1']['y']
            areaa.p2.x = area['p2']['x']
            areaa.p2.y = area['p2']['y']
            areaa.p3.x = area['p3']['x']
            areaa.p3.y = area['p3']['y']
            areaa.p4.x = area['p4']['x']
            areaa.p4.y = area['p4']['y']
            areaa.start.x = area['start']['x']
            areaa.start.y = area['start']['y']
            areaa.end.x = area['end']['x']
            areaa.end.y = area['end']['y']
            areaa.main_wall.start.x = area['wall']['start']['x']
            areaa.main_wall.start.y = area['wall']['start']['y']
            areaa.main_wall.end.x = area['wall']['end']['x']
            areaa.main_wall.end.y = area['wall']['end']['y']
            areaa.main_wall.angle = area['wall']['angle']
            areaa.main_wall.length = area['wall']['length']
            area_array.areas.append(areaa)
        self.get_area = True

    def make_wall_array(self):
    # def make_wall_array(self, wall_array):
        wall_array = WallArray()
        awall = Wall()
        i = 0
        wall_num = len(self.wall_yaml['WALL'])
        print('len wall: ', len(self.wall_yaml['WALL']))
        for i in range(wall_num):
            awall.id = self.wall_yaml['WALL'][i]['id']
            awall.start.x = self.wall_yaml['WALL'][i]['start']['x']
            awall.start.y = self.wall_yaml['WALL'][i]['start']['y']
            awall.end.x = self.wall_yaml['WALL'][i]['end']['x']
            awall.end.y = self.wall_yaml['WALL'][i]['end']['y']
            awall.angle = self.wall_yaml['WALL'][i]['angle']
            awall.length = self.wall_yaml['WALL'][i]['length']
            app_wall = copy.deepcopy(awall)
            wall_array.walls.append(app_wall)
            # awall.id = self.wall_yaml['WALL']['id']
            # awall.start.x = self.wall_yaml['WALL']['start']['x']
            # awall.start.y = wall['start']['y']
            # awall.end.x = wall['end']['x']
            # awall.end.y = wall['end']['y']
            # awall.angle = wall['angle']
            # awall.length = wall['length']
            # wall_array.walls.append(awall)
        self.get_wall = True

        #check
        for wall in self.walls.walls:
            print(wall)

        return wall_array


    def make_edited_map(self):
        if self.get_map:
            print('start edit')
            edited_map = self.map.data
            if self.get_area:
                edited_map = self.edit_map_by_area(self.map, self.areas)
            if self.get_wall:
                edited_map = self.edit_map_by_wall(self.map.data, self.walls)
            new_map = self.map
            new_map.data = edited_map
            self.map_pub.publish(new_map)
            print('=== map edited ===')
        else:
            print('=== map or area is not loaded ===')

    def edit_map_by_area(self, input_map, areas):
        for area in areas.areas:
            edited_map = self.edit_area(input_map, area)
        return edited_map

    def edit_map_by_wall(self, input_map, walls):
        edited_map = list(input_map)
        print('walls: ', walls)
        edited_map = self.edit_wall(edited_map, walls)
        return edited_map

    def edit_area(self, input_map, area):
        # edit map
        # make p1-p2 line , p2-p3 line , p3-p4 line , p4-p1 line
        p12_a , p12_b = self.make_line_param(area.p1.x, area.p1.y, area.p2.x, area.p2.y)
        p23_a , p23_b = self.make_line_param(area.p2.x, area.p2.y, area.p3.x, area.p3.y)
        p34_a , p34_b = self.make_line_param(area.p3.x, area.p3.y, area.p4.x, area.p4.y)
        p41_a , p41_b = self.make_line_param(area.p4.x, area.p4.y, area.p1.x, area.p1.y)
        #new map data(list)
        new_map_data = list(input_map.data)

        # get minimum and maximum index
        p1_index = self.xy_to_index(area.p1.x, area.p1.y)
        p2_index = self.xy_to_index(area.p2.x, area.p2.y)
        p3_index = self.xy_to_index(area.p3.x, area.p3.y)
        p4_index = self.xy_to_index(area.p4.x, area.p4.y)
        min_index = min(p1_index, p2_index, p3_index, p4_index)
        max_index = max(p1_index, p2_index, p3_index, p4_index)

        count = 0
        for i in range(min_index, max_index):
            # get x,y from index
            x, y = self.index_to_xy(i)
            if (self.is_on_line(p12_a,p12_b,x,y) or self.is_on_line(p23_a,p23_b,x,y) or self.is_on_line(p34_a,p34_b,x,y) or self.is_on_line(p41_a,p41_b,x,y)) and self.is_in_area(area,x,y):
                new_map_data[i] = 100
                count += 1
        print ('count: ', count)

        return tuple(new_map_data)

    def make_line_param(self, p1_x, p1_y, p2_x, p2_y):
        a = (p2_y - p1_y) / (p2_x - p1_x)
        b = p1_y - a * p1_x
        return a, b

    def index_to_map_xy(self, index):
        x = index % self.map.info.width
        y = index / self.map.info.width
        return x, y

    def xy_to_index(self, x, y):
        map_x = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
        map_y = int((y - self.map.info.origin.position.y) / self.map.info.resolution)
        print('map_x: ', map_x, 'map_y: ', map_y)
        return map_x + map_y * self.map.info.width

    def index_to_xy(self, index):
        x = index % self.map.info.width * self.map.info.resolution + self.map.info.origin.position.x
        y = index / self.map.info.width * self.map.info.resolution + self.map.info.origin.position.y
        return x, y

    def is_on_line(self, a, b, x, y):
        just_y = a * x + b
        just_x = (y - b) / a
        if abs(just_y - y) < 0.01 or abs(just_x - x) < 0.01:
            return True
        if y == just_y or y == just_y + 1 or y == just_y - 1:
            return True
        else:
            return False

    def is_in_area(self,area,x,y) -> bool:
        #  check if point point is in area
        p1 = area.p1
        p2 = area.p2
        p3 = area.p3
        p4 = area.p4

        # get line param
        line1_a, line1_b = self.make_line_param(p1.x, p1.y, p2.x, p2.y)
        line2_a, line2_b = self.make_line_param(p2.x, p2.y, p3.x, p3.y)
        line3_a, line3_b = self.make_line_param(p3.x, p3.y, p4.x, p4.y)
        line4_a, line4_b = self.make_line_param(p4.x, p4.y, p1.x, p1.y)

        if line1_a * line2_a * line3_a * line4_a == 0 or line1_a * line2_a * line3_a * line4_a == float('inf'):
            x_max = max(area.p1.x,area.p3.x)
            x_min = min(area.p1.x,area.p3.x)
            y_max = max(area.p1.y,area.p3.y)
            y_min = min(area.p1.y,area.p3.y)
            if x_min <= x <= x_max and y_min <= y <= y_max:
                return True
            else:
                return False
        else:
            diff1 = line1_a * x + line1_b - y
            diff2 = line2_a * x + line2_b - y
            diff3 = line3_a * x + line3_b - y
            diff4 = line4_a * x + line4_b - y
            if diff1 * diff2 < 0 and diff3 * diff4 < 0 and diff1 * diff4 > 0 and diff2 * diff3 > 0:
                return True
            else:
                return False

    def edit_wall(self, input_map, walls):
        # if wall pixel is 0, change it to 100

        new_map_data = list(input_map)
        change_index = []
        change_index_list = []
        count = 0
        wall_num = len(walls.walls)
        for i in range(wall_num):
            print('wall', i, ': ', walls.walls[i])
            wall = walls.walls[i]
            # get minimum and maximum index
            start_index = self.xy_to_index(wall.start.x, wall.start.y)
            end_index = self.xy_to_index(wall.end.x, wall.end.y)
            min_index = min(start_index, end_index)
            max_index = max(start_index, end_index)
            min_x = min(wall.start.x, wall.end.x)
            max_x = max(wall.start.x, wall.end.x)
            min_y = min(wall.start.y, wall.end.y)
            max_y = max(wall.start.y, wall.end.y)

            wall_a, wall_b = self.make_line_param(wall.start.x, wall.start.y, wall.end.x, wall.end.y)

            for j in range(min_index, max_index):
                x, y = self.index_to_xy(j)
                if((self.is_on_line(wall_a,wall_b,x,y) and (min_x <= x <= max_x) and (min_y <= y <= max_y)) and input_map[j] == 0):
                # if((self.is_on_line(wall_a,wall_b,x,y) and (min_x <= x <= max_x) and (min_y <= y <= max_y)) and new_map_data[j] == 0):
                    # input_map[j] = 100
                    # new_map_data[j] = 100
                    change_index.append(j)
            change_index_list.append(change_index)
            print('len_chanage index ', len(change_index_list[i]))
            count += 1
        print('count: ', count)
        for i in range(len(change_index_list)):
            for j in range(len(change_index_list[i])):
                new_map_data[change_index_list[i][j]] = 100
        return tuple(new_map_data)
        # return new_map_data


    def main(self):
        while not rospy.is_shutdown():
            if self.get_map:
            # if self.get_map and self.get_area:
                self.make_edited_map()
                self.get_map = False
                self.get_area = False
            rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('map_editor')
    map_editor = MapEditor()
    map_editor.main()
