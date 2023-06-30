#!/usr/bin/env python3

## yarinaoshi ##

## wall = area line

import rospy
import yaml
import math

from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray

from route_maker.msg import Area, AreaArray, Wall, WallArray

class PathMaker:
    def __init__(self):
        rospy.init_node('path_maker')
        print('=== Path Maker ===')

        ## set param
        self.get_map = False
        self.get_area = False
        self.get_wall = False

        self.input_area = AreaArray()
        self.input_wall = WallArray()

        self.nodes = []
        self.shorter_side_angle = 0
        self.path_dict = {}
        self.path_dict['PATH_FRAME'] = []
        self.path_dict['PATH_FRAME'].append('map')
        self.path_dict['OCC_MAP_NAME'] = []
        self.path_dict['OCC_MAP_NAME'].append('map')
        self.path_dict['MAP_DIRECTION'] = []
        self.path_dict['MAP_DIRECTION'].append('0.0')
        self.path_dict['NODE'] = []
        self.path_dict['EDGE'] = []

        self.map_2d = [[]]
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0

        ## check param
        self.area_yaml_file = rospy.get_param('~AREA_YAML_FILE', 'area.yaml')
        self.wall_yaml_file = rospy.get_param('~WALL_YAML_FILE', 'wall.yaml')
        self.path_yaml_file = rospy.get_param('~PATH_YAML_FILE', 'path.yaml')
        ## load yaml file
        self.area_yaml = self.load_area()
        self.wall_yaml = self.load_wall()

        ## make array
        self.make_area_array(self.input_area)
        self.make_wall_array(self.input_wall)

        self.hz = rospy.get_param('~HZ',10)
        self.dist_from_wall = rospy.get_param('~DIST_FROM_WALL',0.2)
        self.brush_width = rospy.get_param('~BRUSH_WIDTH',0.4)
        self.brush_length = rospy.get_param('~BRUSH_LENGTH',0.6)
        self.line_space = rospy.get_param('~LINE_SPACE',0.5)

        ## subscriber
        self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.map_callback, queue_size=1)

        ## publisher
        self.pub_path_marker = rospy.Publisher('/path', MarkerArray, queue_size=1)
        self.pub_nodes_marker = rospy.Publisher('/nodes', MarkerArray, queue_size=1)


    def map_callback(self, msg):
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

    def get_line(self,point1,point2):
        # calculate line equation from two points
        # y = ax + b
        # return a,b
        if point1.x == point2.x:
            a = float('inf')
            b = point1.x
        else:
            a = (point2.y - point1.y) / (point2.x - point1.x)
            b = point1.y - a * point1.x
        return math.floor(a*100)/100,math.floor(b*100)/100

    def pl_dist(self,point,a,b):
        # calculate distance from point to line
        # return distance
        if a == float('inf'):
            dist = abs(point.x - b)
        else:
            dist = abs(a * point.x - point.y + b) / math.sqrt(a ** 2 + 1)
        return dist

    def pp_dist(self,point1,point2):
        return math.sqrt((point2.y-point1.y)**2 + (point2.x-point1.x)**2)

    def check_main_wall_side(self,area):
        # check main wall side
        # return 2 point and dist from area_side
        points = [area.p1,area.p2,area.p3,area.p4,area.p1]
        min_dist = float('inf')
        wall_a, wall_b = self.get_line(area.main_wall.start,area.main_wall.end)
        point1 = Point()
        point2 = Point()

        for i in range(4):
            line_a, line_b = self.get_line(points[i],points[i+1])
            if wall_a == line_a:
                dist = self.pl_dist(area.main_wall.start,line_a,line_b)
                if dist < min_dist:
                    min_dist = dist
                    point1 = points[i]
                    point2 = points[i+1]

        return point1,point2,min_dist

    def set_inside_points(self,area):
        # set inside points of corners
        # return 4 points
        points = [area.p1,area.p2,area.p3,area.p4,area.p1]
        inside_points = [Point() for i in range(4)]
        for i in range(4):
            angle = math.atan2(points[i+1].y-points[i].y,points[i+1].x-points[i].x)
            inside_points[i].x = points[i].x + self.dist_from_wall * math.cos(angle-math.pi/2)
            inside_points[i].y = points[i].y + self.dist_from_wall * math.sin(angle-math.pi/2)

            moved = Point()
            moved.x = inside_points[i].x + self.dist_from_wall * math.cos(angle)
            moved.y = inside_points[i].y + self.dist_from_wall * math.sin(angle)
            if self.is_in_area(area,moved):
                inside_points[i] = moved
                # print('moved1:angle=',angle)
            else :
                inside_points[i].x = points[i].x + self.dist_from_wall * math.cos(angle+math.pi/2)
                inside_points[i].y = points[i].y + self.dist_from_wall * math.sin(angle+math.pi/2)
                moved.x = inside_points[i].x + self.dist_from_wall * math.cos(angle)
                moved.y = inside_points[i].y + self.dist_from_wall * math.sin(angle)
                # print('moved2:angle=',angle)
                # if not self.is_in_area(area,moved):
                #     moved.x = inside_points[i].x - self.dist_from_wall * math.cos(angle)
                #     moved.y = inside_points[i].y + self.dist_from_wall * math.sin(angle)
                #     print('moved3')
                #     if not self.is_in_area(area,moved):
                #         moved.x = inside_points[i].x - self.dist_from_wall * math.cos(angle)
                #         moved.y = inside_points[i].y - self.dist_from_wall * math.sin(angle)
                #         print('moved4')

                inside_points[i] = moved

        return inside_points

    def is_in_area(self,area,point) -> bool:
        #  check if point point is in area
        p1 = area.p1
        p2 = area.p2
        p3 = area.p3
        p4 = area.p4

        line1_a, line1_b = self.get_line(p1,p2)
        line2_a, line2_b = self.get_line(p3,p4)
        line3_a, line3_b = self.get_line(p1,p4)
        line4_a, line4_b = self.get_line(p2,p3)

        if line1_a * line2_a * line3_a * line4_a == 0 or line1_a * line2_a * line3_a * line4_a == float('inf'):
            x_max = max(area.p1.x,area.p3.x)
            x_min = min(area.p1.x,area.p3.x)
            y_max = max(area.p1.y,area.p3.y)
            y_min = min(area.p1.y,area.p3.y)
            if x_min <= point.x <= x_max and y_min <= point.y <= y_max:
                return True
            else:
                return False
        else:
            diff1 = line1_a * point.x + line1_b - point.y
            diff2 = line2_a * point.x + line2_b - point.y
            diff3 = line3_a * point.x + line3_b - point.y
            diff4 = line4_a * point.x + line4_b - point.y
            if diff1 * diff2 < 0 and diff3 * diff4 < 0:
                return True
            else:
                return False

    def search_closest_point(self,point,points):
        # search closest point from points
        # return point
        min_dist = float('inf')
        closest_point = Point()
        for p in points:
            dist = self.pp_dist(point,p)
            if dist < min_dist:
                min_dist = dist
                closest_point = p
        return closest_point

    def search_farthest_point(self,point,points):
        # search farthest point from points
        # return point
        max_dist = 0
        farthest_point = Point()
        for p in points:
            dist = self.pp_dist(point,p)
            if dist > max_dist:
                max_dist = dist
                farthest_point = p
        return farthest_point

    def search_point(self,point,points):
        # search point from points
        # return point

        for p in points:
            if p.x == point.x and p.y == point.y:
                return p
        return Point()

    def rotate_point(self,point,center,angle):
        # rotate point around center
        # return point
        # angle = math.radians(angle)
        x = point.x - center.x
        y = point.y - center.y
        new_x = x * math.cos(angle) - y * math.sin(angle)
        new_y = x * math.sin(angle) + y * math.cos(angle)
        new_x += center.x
        new_y += center.y
        return Point(new_x,new_y,0)

    def draw_spiral_rectangle(self,start_point,angle,long,line_space):
        # draw spiral rectangle
        # return 5 points
        # draw from (x,y) = (0,0)
        # then, move to start_point
        # last, rotation
        # print("angle = ",angle)
        ax = -1
        ay = 1
        if angle < 0 or angle > math.pi/2:
            ax = 1
        if angle < -math.pi/2:
            ay = -1

        points = [Point() for i in range(5)]
        points[0].x = 0
        points[0].y = 0
        points[1].x = long * ax
        points[1].y = 0
        points[2].x = long * ax
        points[2].y = self.brush_width * ay
        points[3].x = 0
        points[3].y = self.brush_width * ay
        points[4].x = 0
        points[4].y = self.brush_width * (1-line_space) * ay

        for i in range(5):
            points[i].x = points[i].x + start_point.x
            points[i].y = points[i].y + start_point.y

        for i in range(5):
            points[i] = self.rotate_point(points[i],start_point,angle)

        return points

    def shift_point(self,point,angle,dist):
        # shift point
        # return point
        point.x = point.x + dist * math.cos(angle)
        point.y = point.y + dist * math.sin(angle)
        point.z = point.z
        return point

    def sides_to_area(self,sides,area):
        area.p1 = sides[0][0]
        area.p2 = sides[1][0]
        area.p3 = sides[2][0]
        area.p4 = sides[3][0]

    def make_path(self,area):
        sides = [[area.p1,area.p2],[area.p2,area.p3],[area.p3,area.p4],[area.p4,area.p1]]
        p1,p2,dist = self.check_main_wall_side(area)
        wall_side = [p1,p2]
        print("dist: ",dist)
        # set main_wall_side = main_wall
        for i in range(4):
            if sides[i][0] == p1 and sides[i][1] == p2:
                angle = math.atan2(p2.y - p1.y,p2.x - p1.x)
                angle += math.pi/2
                # if angle > 2* math.pi:
                #     angle -= 2* math.pi
                # angle = area.main_wall.angle
                sides[i][0] = self.shift_point(sides[i][0],angle,dist)
                sides[(i+1)%4][0] = self.shift_point(sides[(i+1)%4][0],angle,dist)
                break
        area1 = Area()
        area1 = area
        self.sides_to_area(sides,area1)

        inside_points = self.set_inside_points(area1)
        # print("inside_points",inside_points)
        inside_lines = [[inside_points[0],inside_points[1]],[inside_points[1],inside_points[2]],[inside_points[2],inside_points[3]],[inside_points[3],inside_points[0]]]

        self.nodes.append(area1.start)
        # move along lines
        next_point = self.search_closest_point(area1.start,inside_points)
        self.nodes.append(next_point)
        for i in range(4):
            # self.nodes.append(inside_points[i])
            #search line end point
            for j in range (len (inside_lines)):
                if inside_lines[j][0] == next_point:
                    next_point = inside_lines[j][1]
                    # delete line from inside_lines
                    inside_lines.remove(inside_lines[j])
                    break
            self.nodes.append(next_point)

        # move rectangle
        area2 = Area()
        area2.p1 = inside_points[0]
        area2.p2 = inside_points[1]
        area2.p3 = inside_points[2]
        area2.p4 = inside_points[3]
        in_inside_points = self.set_inside_points(area2)
        # search longer side
        angle = math.atan2(area.p1.y - area.p2.y, area.p1.x - area.p2.x)
        longer_side = self.pp_dist(area2.p1,area2.p2) - self.brush_width / 2
        shorter_side = self.pp_dist(area2.p1,area2.p4) - self.brush_width / 2
        if longer_side < shorter_side:
            angle = math.atan2(area2.p1.y - area2.p4.y, area2.p1.x - area2.p4.x)
            longer_side = self.pp_dist(area2.p1,area2.p4) - self.brush_width / 2
            shorter_side = self.pp_dist(area2.p1,area2.p2) - self.brush_width / 2

        self.shorter_side_angle = angle

        next_point = self.search_closest_point(next_point,in_inside_points)
        self.nodes.append(next_point)

        rect_end = self.search_farthest_point(next_point,in_inside_points)
        rect_corners = [Point() for i in range(5)]
        rect_corners = self.draw_spiral_rectangle(next_point,angle,longer_side,self.line_space)
        rect_num = int((shorter_side) / ((1-self.line_space)*self.brush_width)) - 2
        print("rect_num",rect_num)
        # rect_num = 1
        for i in range(rect_num):
            for i in range(5):
                self.nodes.append(rect_corners[i])
            rect_corners = self.draw_spiral_rectangle(rect_corners[4],angle,longer_side,self.line_space)
        for i in range(5):
            self.nodes.append(rect_corners[i])

        # move remaining areas
        # remain = self.pp_dist(rect_end,rect_corners[2])
        # rect_corners = self.draw_spiral_rectangle(area,rect_corners[4],angle,longer_side,remain,mode)

        # check if end point is in the area
        lp = Point()
        p_counter = 0
        lim = len(self.nodes)
        while not self.is_in_area(area2,self.nodes[-1]):
            lp = self.nodes[-1]
            p_counter += 1
            self.nodes.pop()
        if p_counter > lim:
            self.nodes.append(lp)

        # move to end
        self.nodes.append(area.end)

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
        marker.scale.z = 0.0
        marker.color.a = 0.3
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        for j in range(len(nodes)-1):
            marker.points.append(nodes[j])
            marker.points.append(nodes[(j+1)])

        marker_array.markers.append(marker)

        self.pub_path_marker.publish(marker_array)

    def show_nodes(self,nodes,i):
        print("show_nodes")
        array = MarkerArray()
        # for k in range(i):
        for j in range(len(nodes)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "nodes"
            marker.id = j
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            if 0 < j < 5:
                marker.scale.x = 0.15
                marker.scale.y = 0.15
                marker.scale.z = 0.15
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.6
                marker.color.b = 0.0
            marker.pose.position = nodes[j]
            array.markers.append(marker)
        self.pub_nodes_marker.publish(array)


    def add_to_dict(self,path_dict,nodes,i):
        for j in range(len(nodes)-1):
            node_type = 1
            direction = 0
            command = 1
            if 0 < j < 5:
                node_type = 0
                direction = self.shorter_side_angle
            if j == 0 or j == len(nodes)-1:
                command = 0

            path_dict['NODE'].append({'id':j,
                                      'type':node_type,
                                      'x':nodes[j].x,
                                      'y':nodes[j].y,
                                      'direction':direction})
            if j == len(nodes)-2:
                break
            path_dict['EDGE'].append({'command':command,
                                      'start_node_id':j,
                                      'end_node_id':j+1,
                                      'skippable':'false'})

    def main(self):
        rate = rospy.Rate(self.hz)
        is_update_area = False
        is_update_wall = False
        while not rospy.is_shutdown():

            new_area_array = AreaArray()
            self.area_yaml = self.load_area()
            self.make_area_array(new_area_array)
            # print("old_area_array",self.input_area)
            # print("area_array",new_area_array)
            if new_area_array != self.input_area:
                self.input_area = new_area_array
                is_update_area = True
                print ('update area array')

            new_wall_array = WallArray()
            self.wall_yaml = self.load_wall()
            self.make_wall_array(new_wall_array)
            if new_wall_array != self.input_wall:
                self.input_wall = new_wall_array
                is_update_wall = True
                print ('update wall array')

            if self.get_map and self.get_area and self.get_wall:
                ## make path
                for i in range(len(self.input_area.areas)):
                    # self.fix_area(self.input_area.areas[i],self.map_2d)
                    self.make_path(self.input_area.areas[i])
                    self.show_path(self.nodes,i)
                    self.show_nodes(self.nodes,i)
                    # if is_update_area or is_update_wall:
                        # self.add_to_dict(self.path_dict,self.nodes,i)
                    # self.nodes = []
                    # print("path: ",i)
                # self.pub_path.publish(self.path)
            rate.sleep()

        self.add_to_dict(self.path_dict,self.nodes,i)
        with open(self.path_yaml_file, 'w') as f:
            yaml.dump(self.path_dict, f, sort_keys=False)
        print('shutting down')

if __name__ == '__main__':
    path_maker = PathMaker()
    path_maker.main()
