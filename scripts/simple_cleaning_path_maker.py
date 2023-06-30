#!/usr/bin/env python3

# write node edge pairs (=path) to yaml

import rospy
import yaml
import math

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid

from route_maker.msg import Area, AreaArray, Wall, WallArray, PathArray

from visualization_msgs.msg import Marker, MarkerArray

class SimpleCleaningPathMaker:
    def __init__(self):
        rospy.init_node('simple_cleaning_path_maker')
        print('=== simple_cleaning_path_maker ===')

        self.use_yaml = rospy.get_param('~USE_YAML',True)
        self.get_map = False
        self.get_area = False
        self.get_wall = False
        self.input_area = AreaArray()
        self.input_wall = WallArray()
        self.path = Path()
        self.nodes = []

        self.global_map = OccupancyGrid()
        self.map_2d = [[]]
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0

        ## check param
        if self.use_yaml:
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

        self.hz = rospy.get_param('~HZ',10)
        self.dist_from_wall = rospy.get_param('~DIST_FROM_WALL',0.2)
        self.brush_width = rospy.get_param('~BRUSH_WIDTH',0.2)
        self.brush_length = rospy.get_param('~BRUSH_LENGTH',0.6)
        self.line_space = rospy.get_param('~LINE_SPACE',0.6)

        ## subscriber
        self.sub_global_map = rospy.Subscriber('/map', OccupancyGrid, self.global_map_callback, queue_size=1)
        self.sub_area = rospy.Subscriber('/area', AreaArray, self.area_callback, queue_size=1)
        self.sub_wall = rospy.Subscriber('/wall', WallArray, self.wall_callback, queue_size=1)

        ## publisher
        self.pub_path = rospy.Publisher('/path', MarkerArray, queue_size=1)

    def global_map_callback(self, msg):
        self.global_map = msg
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_2d = [[0 for i in range(self.map_width)] for j in range(self.map_height)]
        print('map_width:',self.map_width)
        print('map_height:',self.map_height)
        print('map_resolution:',self.map_resolution)
        for i in range(self.map_height - 1):
            for j in range(self.map_width - 1):
                self.map_2d[i][j] = msg.data[i * self.map_width + j]


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

    def make_wall_array(self,wall_array):
        walll = Wall()
        for wall in self.wall_yaml['WALL']:
            walll.id = wall['id']
            walll.start.x = wall['start']['x']
            walll.start.y = wall['start']['y']
            walll.end.x = wall['end']['x']
            walll.end.y = wall['end']['y']
            walll.angle = wall['angle']
            walll.length = wall['length']
            wall_array.walls.append(walll)
        self.get_wall = True

    def fix_area(self,area,map2d):
        angle = area.main_wall.angle
        wall_start = area.main_wall.start
        wall_end = area.main_wall.end
        x = wall_start.x
        y = wall_start.y

        ## if main wall line cells are not 100, fix it
        while(x <= wall_end.x or y <= wall_end.y):
            if map2d[int(y)][int(x)] != 100:
                map2d[int(y)][int(x)] = 100
            x += math.cos(angle+math.pi/2)
            y += math.sin(angle+math.pi/2)

        ## if p1-p2 line cells are not 100, fix it
        x = area.p1.x
        y = area.p1.y
        angle = math.atan2(area.p2.y-area.p1.y,area.p2.x-area.p1.x)
        while(x <= area.p2.x or y <= area.p2.y):
            if map2d[int(y)][int(x)] != 100:
                map2d[int(y)][int(x)] = 100
            x += math.cos(angle+math.pi/2)
            y += math.sin(angle+math.pi/2)

        ## if p2-p3 line cells are not 100, fix it
        x = area.p2.x
        y = area.p2.y
        angle = math.atan2(area.p3.y-area.p2.y,area.p3.x-area.p2.x)
        while(x <= area.p3.x or y <= area.p3.y):
            if map2d[int(y)][int(x)] != 100:
                map2d[int(y)][int(x)] = 100
            x += math.cos(angle+math.pi/2)
            y += math.sin(angle+math.pi/2)

        ## if p3-p4 line cells are not 100, fix it
        x = area.p3.x
        y = area.p3.y
        angle = math.atan2(area.p4.y-area.p3.y,area.p4.x-area.p3.x)
        while(x <= area.p4.x or y <= area.p4.y):
            if map2d[int(y)][int(x)] != 100:
                map2d[int(y)][int(x)] = 100
            x += math.cos(angle+math.pi/2)
            y += math.sin(angle+math.pi/2)

        ## if p4-p1 line cells are not 100, fix it
        x = area.p4.x
        y = area.p4.y
        angle = math.atan2(area.p1.y-area.p4.y,area.p1.x-area.p4.x)
        while(x <= area.p1.x or y <= area.p1.y):
            if map2d[int(y)][int(x)] != 100:
                map2d[int(y)][int(x)] = 100
            x += math.cos(angle+math.pi/2)
            y += math.sin(angle+math.pi/2)

        print ("area fixed")

    def run_along_line(self,map2d,start_point,end_point,angle):
        path = Path()
        x = start_point.x
        y = start_point.y
        pose = PoseStamped()
        while(x != end_point.x or y != end_point.y):
            path_x = int(x + self.dist_from_wall*math.cos(angle+math.pi/2))
            path_y = int(y + self.dist_from_wall*math.sin(angle+math.pi/2))
            while map2d[path_y][path_x] == 100:
                path_x += self.dist_from_wall*math.cos(angle-math.pi/2)
                path_y += self.dist_from_wall*math.sin(angle-math.pi/2)

            path_x = int(path_x)
            path_y = int(path_y)
            pose.pose.position.x = path_x*self.map_resolution
            pose.pose.position.y = path_y*self.map_resolution
            path.poses.append(pose)

            x += math.cos(angle)
            y += math.sin(angle)

        return path

    def p_l_distance(self,point,line_start,line_end):
        x0 = point.x
        y0 = point.y
        x1 = line_start.x
        y1 = line_start.y
        x2 = line_end.x
        y2 = line_end.y

        return abs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1))/math.sqrt((x2-x1)**2+(y2-y1)**2)

    def search_nearest_line(self,point,area):
        dist = 100000
        x = [area.p1.x,area.p2.x,area.p3.x,area.p4.x,area.p1.x]
        y = [area.p1.y,area.p2.y,area.p3.y,area.p4.y,area.p1.y]
        nearest_line = []
        # dist_array = []
        # num_array = []
        for i in range(4):
            nyanyanya = self.p_l_distance(point,Point(x[i],y[i],0),Point(x[(i+1)%4],y[(i+1)%4],0))

            if nyanyanya < dist:
                dist = nyanyanya
                nearest_line = [Point(x[i],y[i],0),Point(x[(i+1)%4],y[(i+1)%4],0)]
                pp_dist = math.sqrt((x[i] - point.x)**2 + (y[i] - point.y)**2)
                np_dist = math.sqrt((x[(i+1)%4] - point.x)**2 + (y[(i+1)%4] - point.y)**2)
                if pp_dist > np_dist:
                    nearest_line = [Point(x[i],y[i],0),Point(x[(i+1)%4],y[(i+1)%4],0)]
                else:
                    nearest_line = [Point(x[(i+1)%4],y[(i+1)%4],0),Point(x[i],y[i],0)]
                num = i
        return nearest_line,num

    def go_straight(self,start_point,end_point,angle,map2d):
        path = Path()
        path.header.frame_id = "map"
        x = start_point.x
        y = start_point.y
        pose = PoseStamped()
        while(x != end_point.x or y != end_point.y):
            path_x = int(x + self.dist_from_wall*math.cos(angle-math.pi/2))
            path_y = int(y + self.dist_from_wall*math.sin(angle-math.pi/2))
            while map2d[path_y][path_x] == 100:
                path_x += self.dist_from_wall*math.cos(angle-math.pi/2)
                path_y += self.dist_from_wall*math.sin(angle-math.pi/2)

            path_x = int(path_x)
            path_y = int(path_y)
            pose.pose.position.x = path_x*self.map_resolution
            pose.pose.position.y = path_y*self.map_resolution
            path.poses.append(pose)

            x += math.cos(angle)
            y += math.sin(angle)

        return path

    def set_wall_side_point(self,point,angle,mode):
        x = point.x
        y = point.y
        x += self.dist_from_wall*math.cos(angle-math.pi/2)
        y += self.dist_from_wall*math.sin(angle-math.pi/2)
        if mode == 1:
            x += self.dist_from_wall*math.cos(-angle)
            y += self.dist_from_wall*math.sin(-angle)
        if mode == 2:
            x += self.dist_from_wall*math.cos(-angle)
            y += -self.dist_from_wall*math.sin(-angle)
        # while map2d[int(y)][int(x)] == 100:
        #     x += self.dist_from_wall*math.cos(self.wall_angle-math.pi/2)
        #     y += self.dist_from_wall*math.sin(self.wall_angle-math.pi/2)
        return Point(x,y,0)


    def make_path(self,wall_array,area):
        mini_path = Path()
        wall_in_area = WallArray()
        main_wall = Wall()
        main_wall = area.main_wall
        wall_start = area.main_wall.start
        wall_end = area.main_wall.end
        start_point = Point()
        end_point = Point()
        start_point = area.start
        end_point = area.end
        corners = [area.p1,area.p2,area.p3,area.p4]
        passed_line = []
        dist_array = []
        ## search wall in the area
        self.search_wall(wall_array,area,wall_in_area)
        ## make path
        ## start from start point
        ## first move to the wall start point
        self.nodes.append(start_point)
        sd = math.sqrt((start_point.x - main_wall.start.x)**2 + (start_point.y - main_wall.start.y)**2)
        ed = math.sqrt((end_point.x - main_wall.start.x)**2 + (end_point.y - main_wall.start.y)**2)
        if sd > ed:
            wall_start = area.main_wall.end
            wall_end = area.main_wall.start

        # angle = math.atan2(start_point.y - main_wall.start.y,start_point.x - main_wall.start.x)
        angle = math.atan2(wall_end.y - wall_start.y,wall_end.x - wall_start.x)
        self.nodes.append(self.set_wall_side_point(wall_start,angle,0))
        ## move to the wall end point
        self.nodes.append(self.set_wall_side_point(wall_end,angle,1))
        # ## move along next line(nearest_line)
        # ## if nearest_line has already been passed, move to the next nearest line
        # ## if nearest_line is the last line, move to the end point
        nearest_line,num = self.search_nearest_line(self.nodes[2],area)
        # angle = math.atan2(self.nodes[2].y - nearest_line[0].y,self.nodes[2].x - nearest_line[0].x)
        angle = math.atan2(nearest_line[0].y - self.nodes[2].y,nearest_line[0].x - self.nodes[2].x)
        self.nodes.append(self.set_wall_side_point(nearest_line[0],angle,2))
        next_corner = corners[(num+1)%4]
        if next_corner == nearest_line[1]:
            next_corner = corners[(num+3)%4]
        angle = math.atan2(next_corner.y - nearest_line[0].y,next_corner.x - nearest_line[0].x)
        # angle = math.atan2(nearest_line[0].y - next_corner.y,nearest_line[0].x - next_corner.x)
        self.nodes.append(self.set_wall_side_point(next_corner,angle,1))
        # ## back to wall start point , move along next line
        # # angle = math.atan2(next_corner.y - main_wall.start.y,next_corner.x - main_wall.start.x)
        angle = math.atan2(main_wall.start.y - next_corner.y,main_wall.start.x - next_corner.x)
        self.nodes.append(self.set_wall_side_point(wall_start,angle,))
        #
        # ## move inside the area
        # ## draw rectangle
        #
        # ## move to the end point
        # self.nodes.append(end_point)

        ## move along the wall
        # mini_path = self.go_straight(start_point,main_wall.start,main_wall.angle,map2d)
        # self.path.poses.extend(mini_path.poses)
        ## move along the wall
        # self.run_along_line(self.path,map2d,main_wall.start,main_wall.end,main_wall.angle)

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

    def show_path(self,nodes,i):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path"
        marker.id = i
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for i in range(len(nodes)-1):
            marker.points.append(nodes[i])
            marker.points.append(nodes[(i+1)])

        marker_array.markers.append(marker)

        self.pub_path.publish(marker_array)

    def main(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            # print("use_yaml: ",self.use_yaml)
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

            if self.get_map and self.get_area and self.get_wall:
                ## make path
                for i in range(len(self.input_area.areas)):
                    # self.fix_area(self.input_area.areas[i],self.map_2d)
                    self.make_path(self.input_wall,self.input_area.areas[i])
                    self.show_path(self.nodes,i)
                    self.nodes = []
                    # print("path: ",i)
                #     self.make_path(self.input_wall.walls[i],self.input_area.areas[i])
                # self.pub_path.publish(self.path)

        rate.sleep()

if __name__ == '__main__':
    simple_cleaning_path_maker = SimpleCleaningPathMaker()
    simple_cleaning_path_maker.main()

