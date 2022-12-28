#!/usr/bin/env python

import rospy
import yaml
import math

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from jsk_rviz_plugins.msg import OverlayText

from route_maker.msg import Wall, WallArray

class WallRecoder:
    def __init__(self):
        rospy.init_node('wall_recorder')
        print('=== Wall Recorder ===')

        ## check param
        if rospy.has_param('~FILE_NAME'):
            self.file_name = rospy.get_param('~FILE_NAME')
            print ('file name: ' + self.file_name)

        self.points = []
        self.wall = Wall()
        self.wall_array = WallArray()
        self.wall_dict = {}
        self.wall_dict['WALL'] = []
        self.id = 0
        self.guide = OverlayText()

        ## subscriber
        self.sub = rospy.Subscriber('clicked_point', PointStamped, self.callback)

        ## publisher
        self.pub_wall = rospy.Publisher('wall', WallArray, queue_size=1)
        ## visualization
        self.pub_clicked_point = rospy.Publisher('clicked_point_marker', Marker, queue_size=1)
        self.pub_wall_marker = rospy.Publisher('wall_marker', MarkerArray, queue_size=1)
        self.pub_guide = rospy.Publisher('wall_recoder_guide', OverlayText, queue_size=1)

        self.init_guide()

    def init_guide(self):
        self.guide.text = 'Click Start Point of Wall'
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

    def callback(self,msg):
        x = round(msg.point.x, 3)
        y = round(msg.point.y, 3)
        z = round(msg.point.z, 3)
        clicked = Point(x,y,z)
        self.points.append(clicked)

        if len(self.points) and len(self.points) % 2 == 1:
            self.show_clicked_point(clicked)
            self.guide.text = 'Click End point of Wall'

        if len(self.points) and len(self.points) % 2 == 0:
            self.show_clicked_point(clicked)
            self.wall.id = self.id
            self.wall.start = self.points[2*self.id]
            self.wall.end = clicked
            angle = math.atan2(self.wall.end.y - self.wall.start.y, self.wall.end.x - self.wall.start.x)
            self.wall.angle = round(angle, 3)
            length = math.sqrt((self.wall.end.x - self.wall.start.x)**2 + (self.wall.end.y - self.wall.start.y)**2)
            self.wall.length = round(length, 3)
            self.wall_array.walls.append(self.wall)
            self.show_walls(self.wall_array)
            self.add_to_dict(self.wall)

            self.guide.text = 'Recode Success! Click Start Point of Another Wall'
            self.pub_wall.publish(self.wall_array)

            self.id += 1

        self.pub_guide.publish(self.guide)

    def add_to_dict(self, wall):
        self.wall_dict['WALL'].append({'id': wall.id,
                                        'start': {'x': wall.start.x, 'y': wall.start.y},
                                        'end': {'x': wall.end.x, 'y': wall.end.y},
                                        'angle': wall.angle,
                                        'length': wall.length})

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


    def show_walls(self, wall_array):
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

    def main(self):
        # self.init_guide()
        rospy.spin()
        if rospy.is_shutdown():
            with open(self.file_name, 'w') as f:
                yaml.dump(self.wall_dict, f, sort_keys=False)
            print('shutting down')

if __name__ == '__main__':
    wall_recoder = WallRecoder()
    wall_recoder.main()





