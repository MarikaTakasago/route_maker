#!/usr/bin/env python3

## Area recoder flow ##
## 1.decide main wall
## 2.click area start point and end point
## **set area parallel to main wall**
## 3.decide cleaning start point
## 4.decide cleanin end point
## 5.record area

import rospy
import yaml
import math

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from jsk_rviz_plugins.msg import OverlayText

from route_maker.msg import Area, AreaArray, Wall, WallArray


class AreaRecoder:
    def __init__(self):
        rospy.init_node('area_recoder')
        print('=== Area Recoder ===')

        ## check param
        # if rospy.has_param('~PARAM_NAME'):
        #     self.param = rospy.get_param('~PARAM_NAME')
        if rospy.has_param('~FILE_NAME'):
            self.file_name = rospy.get_param('~FILE_NAME')
            print('file_name: ' + self.file_name)

        self.display = rospy.get_param('~DISPLAY',True)

        self.points = []
        self.area = Area()
        self.area_array = AreaArray()
        self.area_dict = {}
        self.area_dict['AREA'] = []
        self.id = 0
        self.start_end_points = []
        self.wall = Wall()
        self.wall_array = WallArray()
        self.guide = OverlayText()


        ## subscriber
        self.sub = rospy.Subscriber('clicked_point', PointStamped, self.callback)

        ## publisher
        self.pub_area = rospy.Publisher('area', AreaArray, queue_size=1)
        ## visualization
        self.pub_clicked_point = rospy.Publisher('clicked_point_marker', Marker, queue_size=1)
        self.pub_area_marker = rospy.Publisher('area_marker', MarkerArray, queue_size=1)
        self.pub_start_end_points = rospy.Publisher('start_end_point_marker', MarkerArray, queue_size=1)
        self.pub_wall_marker = rospy.Publisher('wall_marker', MarkerArray, queue_size=1)
        self.pub_guide = rospy.Publisher('area_recoder_guide', OverlayText, queue_size=1)

        self.init_guide()

    def init_guide(self):
        self.guide.text = 'Click Start Point of Area'
        self.guide.width = 650
        self.guide.height = 30
        self.guide.left = 0
        self.guide.top = 0
        self.guide.line_width = 2
        self.guide.text_size = 15
        self.guide.font = 'DejaVu Sans Mono'
        # forground color: cyan
        self.guide.fg_color.r = 0.0
        self.guide.fg_color.g = 1.0
        self.guide.fg_color.b = 1.0
        self.guide.fg_color.a = 1.0
        # background color: black, a = 0.5
        self.guide.bg_color.r = 0.0
        self.guide.bg_color.g = 0.0
        self.guide.bg_color.b = 0.0
        self.guide.bg_color.a = 0.5

        self.pub_guide.publish(self.guide)

    def callback(self, msg):
        x = round(msg.point.x,3)
        y = round(msg.point.y,3)
        z = round(msg.point.z,3)
        clicked = Point(x,y,z)
        # clicked.x = x
        # clicked.y = y
        # clicked.z = z
        self.points.append(clicked)

        if len(self.points) % 6 == 1 or (len(self.points) and len(self.points) % 6 == 2):
            self.show_clicked_point(clicked)
            self.make_wall(clicked, len(self.points) % 6)

        elif len(self.points) and len(self.points) % 6 == 3:
            self.show_clicked_point(clicked)
            self.guide.text = 'Click End Point of Area'
            print('click another point')

        elif len(self.points) and len(self.points) % 6 == 4:
            self.show_clicked_point(clicked)
            self.area.id = self.id
            self.make_area(self.points[6*self.id+2],clicked,self.area)
            # self.area.p1 = self.points[6*self.id]
            # self.area.p3 = clicked
            # self.area.p2.x = self.area.p3.x
            # self.area.p2.y = self.area.p1.y
            # self.area.p2.z = self.area.p1.z
            # self.area.p4.x = self.area.p1.x
            # self.area.p4.y = self.area.p3.y
            # self.area.p4.z = self.area.p1.z

            self.show_area(self.area)

            self.guide.text = 'Click Start Point of Cleaning Route'
            print('click start point')

        elif len(self.points) and len(self.points) % 6 == 5:
            if self.is_in_area(self.area,clicked):
                self.area.start = clicked
                self.start_end_points.append(clicked)
                self.show_clicked_points(self.start_end_points)

                self.guide.text = 'Click End Point of Cleaning Route'
                print('click end point')

            else:
                self.show_clicked_point(clicked)
                self.guide.text = 'Clicked Point is not in Areas'
                print('start point is not in area')
                self.points.pop(len(self.points)-1)

        elif len(self.points) and len(self.points) % 6 == 0:
            if self.is_in_area(self.area,clicked):
                self.area.end = clicked
                self.start_end_points.append(clicked)
                self.show_clicked_points(self.start_end_points)

                self.add_to_dict(self.area)
                self.area_array.areas.append(self.area)
                self.pub_area.publish(self.area_array)
                self.id += 1

                self.guide.text = 'Recode Success! Click Start Point of Next MainWall'
                print('area recoded')

            else:
                self.show_clicked_point(clicked)
                self.guide.text = 'Clicked Point is not in Areas'
                print('end point is not in area')
                self.points.pop(len(self.points)-1)

        self.pub_guide.publish(self.guide)

    def make_wall(self, clicked, num):
        if num == 1:
            self.wall.id = 0
            self.wall.start = clicked

            self.guide.text = 'Click End Point of MainWall'

        elif num == 2:
            self.wall.end = clicked

            angle = math.atan2(self.wall.end.y - self.wall.start.y, self.wall.end.x - self.wall.start.x)
            length = math.sqrt((self.wall.end.y - self.wall.start.y)**2 + (self.wall.end.x - self.wall.start.x)**2)
            self.wall.angle = angle
            self.wall.length = length

            self.area.main_wall = self.wall
            self.wall_array.walls.append(self.wall)
            self.show_walls(self.wall_array)
            self.wall = Wall()

            self.guide.text = 'MainWall Recoded. Click Start Point of the Area'

    def make_area(self, p1, p3, area):
        #line p1 to p2 is parallel to main_wall
        area.p1 = p1
        area.p3 = p3
        wall_start = self.area.main_wall.start
        wall_end = self.area.main_wall.end
        wall_a, wall_b = self.get_line(wall_start, wall_end)

        if wall_a == 0:
            area.p2.x = p1.x
            area.p2.y = p3.y
            area.p2.z = p1.z
            area.p4.x = p3.x
            area.p4.y = p1.y
            area.p4.z = p1.z
            return

        if wall_a == float('inf'):
            area.p2.x = p3.x
            area.p2.y = p1.y
            area.p2.z = p1.z
            area.p4.x = p1.x
            area.p4.y = p3.y
            area.p4.z = p1.z
            return

        p2 = Point()
        p1p2_b = p1.y - wall_a * p1.x
        p2p3_a = -1 / wall_a
        p2p3_b = p3.y - p2p3_a * p3.x

        # print('line p1p2: y = {}x + {}'.format(wall_a, p1p2_b))
        # print('line p2p3: y = {}x + {}'.format(p2p3_a, p2p3_b))

        if wall_a == 1 or wall_a == -1:
            p2.y = (p1p2_b + p2p3_b) / 2
            p2.x = (p1p2_b - p2.y) / wall_a
        else :
            p2.x = (p2p3_b - p1p2_b) / (wall_a - p2p3_a)
            p2.y = wall_a * p2.x + p1p2_b
        p2.z = p1.z

        p4 = Point()
        p3p4_b = p3.y - wall_a * p3.x
        p4p1_a = -1 / wall_a
        p4p1_b = p1.y - p4p1_a * p1.x
        if wall_a != 1 and wall_a != -1:
            p4.x = (p4p1_b - p3p4_b) / (wall_a - p4p1_a)
            p4.y = wall_a * p4.x + p3p4_b
        else :
            p4.y = (p3p4_b + p4p1_b) / 2
            p4.x = (p3p4_b - p4.y) / wall_a
        p4.z = p1.z

        area.p2 = p2
        area.p4 = p4

    def is_cross(self, p1, p2, p3, p4):
        # check if line segment p1 to p2 is cross line p3 to p4
        # if p1 and p2 are on the same side of line p3 to p4, or p3 and p4 are on the same side of line p1 to p2, return False

        line1_a, line1_b = self.get_line(p1,p2)
        line2_a, line2_b = self.get_line(p3,p4)

        line1_xmin = min(p1.x,p2.x)
        line1_xmax = max(p1.x,p2.x)
        line1_ymin = min(p1.y,p2.y)
        line1_ymax = max(p1.y,p2.y)

        line2_ymin = min(p3.y,p4.y)
        line2_yminx = p3.x
        if line2_ymin == p4.y:
            line2_yminx = p4.x
        line2_ymax = max(p3.y,p4.y)
        line2_ymaxx = p3.x
        if line2_ymax == p4.y:
            line2_ymaxx = p4.x

        # check side
        if line1_a * line2_ymaxx + line1_b < line2_ymax and line1_a * line2_yminx + line1_b > line2_ymin:
            #check cross point
            cross_x = (line2_b - line1_b) / (line1_a - line2_a)
            if line1_xmin <= cross_x <= line1_xmax:
                return True
            else :
                return False
        else:
            return False

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
        return a,b

    def add_to_dict(self,area):
        #  make dict for yaml
        self.area_dict['AREA'].append({'id':area.id,
                                        'p1':{'x':area.p1.x,'y':area.p1.y},
                                        'p2':{'x':area.p2.x,'y':area.p2.y},
                                        'p3':{'x':area.p3.x,'y':area.p3.y},
                                        'p4':{'x':area.p4.x,'y':area.p4.y},
                                        'start':{'x':area.start.x,'y':area.start.y},
                                        'end':{'x':area.end.x,'y':area.end.y},
                                        'wall':{'start':{'x':area.main_wall.start.x,'y':area.main_wall.start.y},
                                                'end':{'x':area.main_wall.end.x,'y':area.main_wall.end.y},
                                                'angle':area.main_wall.angle,
                                                'length':area.main_wall.length
                                                }
                                        })

    def show_clicked_points(self,clicked_points):
        ## yellow and pink dict
        color_dict = {0:{'r':1.0,'g':1.0,'b':0.0},1:{'r':1.0,'g':0.0,'b':1.0}}
        array = MarkerArray()
        for i in range(len(clicked_points)):

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "clicked_points"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = color_dict[i%2]['r']
            marker.color.g = color_dict[i%2]['g']
            marker.color.b = color_dict[i%2]['b']
            marker.pose.position.x = clicked_points[i].x
            marker.pose.position.y = clicked_points[i].y
            marker.pose.position.z = clicked_points[i].z
            array.markers.append(marker)

        self.pub_start_end_points.publish(array)

    def show_clicked_point(self,clicked):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "clicked_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.1
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.position.x = clicked.x
        marker.pose.position.y = clicked.y
        marker.pose.position.z = clicked.z
        self.pub_clicked_point.publish(marker)

    def show_area(self,area):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "area"
        marker.id = area.id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        marker.points.append(area.p1)
        marker.points.append(area.p2)

        marker.points.append(area.p2)
        marker.points.append(area.p3)

        marker.points.append(area.p3)
        marker.points.append(area.p4)

        marker.points.append(area.p4)
        marker.points.append(area.p1)

        marker_array = MarkerArray()
        marker_array.markers.append(marker)

        self.pub_area_marker.publish(marker_array)

    def show_walls(self,wall_array):
        marker_array = MarkerArray()
        i = 0
        for wall in wall_array.walls:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "wall"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.points.append(wall.start)
            marker.points.append(wall.end)
            marker_array.markers.append(marker)
            i += 1

        self.pub_wall_marker.publish(marker_array)

    def is_in_area(self,area,clicked) -> bool:
        #  check if clicked point is in area
        # line1 = (area.p1,area.p2)
        # line2 = (area.p3,area.p4)
        # line3 = (area.p1,area.p4)
        # line4 = (area.p2,area.p3)
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
            if x_min <= clicked.x <= x_max and y_min <= clicked.y <= y_max:
                return True
            else:
                return False
        else:
            diff1 = line1_a * clicked.x + line1_b - clicked.y
            diff2 = line2_a * clicked.x + line2_b - clicked.y
            diff3 = line3_a * clicked.x + line3_b - clicked.y
            diff4 = line4_a * clicked.x + line4_b - clicked.y
            if diff1 * diff2 < 0 and diff3 * diff4 < 0:
                return True
            else:
                return False

    def main(self):
        self.init_guide()
        rospy.spin()
        if rospy.is_shutdown():
            with open(self.file_name, 'w') as f:
                yaml.dump(self.area_dict, f, sort_keys=False)
            print('shutting down')

if __name__ == '__main__':
    area_recoder = AreaRecoder()

    area_recoder.main()
