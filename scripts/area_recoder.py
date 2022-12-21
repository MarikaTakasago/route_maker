#!/usr/bin/env python3

import rospy
import yaml

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from jsk_rviz_plugins.msg import OverlayText

from route_maker.msg import Area, AreaArray


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

        self.points = []
        self.area = Area()
        self.area_array = AreaArray()
        self.area_dict = {}
        self.area_dict['AREA'] = []
        self.id = 0
        self.start_end_points = []
        self.guide = OverlayText()

        ## subscriber
        self.sub = rospy.Subscriber('clicked_point', PointStamped, self.callback)

        ## publisher
        self.pub_area = rospy.Publisher('area', AreaArray, queue_size=1)
        ## visualization
        self.pub_clicked_point = rospy.Publisher('clicked_point_marker', Marker, queue_size=1)
        self.pub_area_marker = rospy.Publisher('area_marker', MarkerArray, queue_size=1)
        self.pub_start_end_points = rospy.Publisher('start_end_point_marker', MarkerArray, queue_size=1)
        self.pub_guide = rospy.Publisher('area_recoder_guide', OverlayText, queue_size=1)

    def init_guide(self):
        self.guide.text = 'Click Start Point of Area'
        self.guide.width = 600
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

        if len(self.points) % 4 == 1:
            self.show_clicked_point(clicked)
            self.guide.text = 'Click End Point of Area'
            print('click another point')

        elif len(self.points) and len(self.points) % 4 == 2:
            self.show_clicked_point(clicked)
            self.area.id = self.id
            self.area.p1 = self.points[4*self.id]
            self.area.p3 = clicked
            self.area.p2.x = self.area.p3.x
            self.area.p2.y = self.area.p1.y
            self.area.p2.z = self.area.p1.z
            self.area.p4.x = self.area.p1.x
            self.area.p4.y = self.area.p3.y
            self.area.p4.z = self.area.p1.z

            self.show_area(self.area)

            self.guide.text = 'Click Start Point of Cleaning Route'
            print('click start point')

        elif len(self.points) and len(self.points) % 4 == 3:
            if self.is_in_area(self.area,clicked):
                self.area.start = clicked
                self.start_end_points.append(clicked)
                self.show_clicked_points(self.start_end_points)

                self.guide.text = 'Click End Point of Cleaning Route'
                print('click end point')

            else:
                self.show_clicked_point(clicked)
                self.guide.text = 'Clicked Point is not in Area'
                print('start point is not in area')
                self.points.pop(len(self.points)-1)

        elif len(self.points) and len(self.points) % 4 == 0:
            if self.is_in_area(self.area,clicked):
                self.area.end = clicked
                self.start_end_points.append(clicked)
                self.show_clicked_points(self.start_end_points)

                self.add_to_dict(self.area)
                self.area_array.areas.append(self.area)
                self.pub_area.publish(self.area_array)
                self.id += 1

                self.guide.text = 'Recode Success! Click Start Point of Another Area'
                print('area recoded')

            else:
                self.show_clicked_point(clicked)
                self.guide.text = 'Clicked Point is not in Area'
                print('end point is not in area')
                self.points.pop(len(self.points)-1)

        self.pub_guide.publish(self.guide)


    def add_to_dict(self,area):
        #  make dict for yaml
        self.area_dict['AREA'].append({'id':area.id,
                                        'p1':{'x':area.p1.x,'y':area.p1.y},
                                        'p2':{'x':area.p2.x,'y':area.p2.y},
                                        'p3':{'x':area.p3.x,'y':area.p3.y},
                                        'p4':{'x':area.p4.x,'y':area.p4.y},
                                        'start':{'x':area.start.x,'y':area.start.y},
                                        'end':{'x':area.end.x,'y':area.end.y}})

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
            marker.scale.x = 0.5
            marker.scale.y = 0.5
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
        marker.scale.x = 0.5
        marker.scale.y = 0.5
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

    def is_in_area(self,area,clicked) -> bool:
        x_max = max(area.p1.x,area.p3.x)
        x_min = min(area.p1.x,area.p3.x)
        y_max = max(area.p1.y,area.p3.y)
        y_min = min(area.p1.y,area.p3.y)

        if x_min <= clicked.x <= x_max and y_min <= clicked.y <= y_max:
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
