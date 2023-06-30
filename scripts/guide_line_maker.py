#!/usr/bin/env python3

import rospy
import yaml
import math
from geometry_msgs.msg import PoseStamped ,PoseArray ,Point
from visualization_msgs.msg import Marker , MarkerArray

class MakeGuideLine:
    class wall_line:
        def __init__(self, x1, y1, x2, y2,angle):
            self.x1 = x1
            self.y1 = y1
            self.x2 = x2
            self.y2 = y2
            self.angle = angle

    def __init__(self):
        rospy.init_node('make_guide_line')
        print('=== make_guide_line ===')

        ## check param
        if rospy.has_param('~WALL_FILE'):
            self.wall_file = rospy.get_param('~WALL_FILE')
        if rospy.has_param('~WALL_DIST'):
            self.wall_dist = rospy.get_param('~WALL_DIST')

        self.hz = rospy.get_param('~HZ', 10)

        self.wall_num = 0
        self.wall_line_list = []

        ## load wall yaml
        self.wall_yaml = self.load_data()

        ## make list
        self.make_list(self.wall_line_list)

        ## publisher
        self.pub_wall = rospy.Publisher('wall_line', MarkerArray, queue_size=1)
        self.pub_guide = rospy.Publisher('guide_line', MarkerArray, queue_size=1)

    def load_data(self):
        with open(self.wall_file, 'r') as f:
            wall_yaml = yaml.safe_load(f)
        return wall_yaml

    def make_list(self, wall_list):
        ## list of wall
        for wall in self.wall_yaml['wall_lines']:
            angle = math.atan2(wall[3] - wall[1], wall[2] - wall[0])
            wall_list.append(self.wall_line(wall[0], wall[1], wall[2], wall[3], angle))
            print('angle: ', angle)
            self.wall_num += 1

    def make_wall_marker(self, wall_line_list):
        marker_array = MarkerArray()
        for i in range(self.wall_num):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "guide_line"
            marker.id = i
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.points.append(Point(self.wall_line_list[i].x1, self.wall_line_list[i].y1,0))
            marker.points.append(Point(self.wall_line_list[i].x2, self.wall_line_list[i].y2,0))
            marker_array.markers.append(marker)

        return marker_array
        print('=== make_wall_line ===')

    def make_guide_line(self,wall_line_list):
        ## make new array of guide line on 'wall_dist' from wall
        guide_line_list = []
        for i in range(self.wall_num):
            guide_line_list.append(self.wall_line
                                   (wall_line_list[i].x1 + self.wall_dist * math.cos(self.wall_line_list[i].angle - math.pi / 2),
                                    wall_line_list[i].y1 + self.wall_dist * math.sin(self.wall_line_list[i].angle - math.pi / 2),
                                    wall_line_list[i].x2 + self.wall_dist * math.cos(self.wall_line_list[i].angle - math.pi / 2),
                                    wall_line_list[i].y2 + self.wall_dist * math.sin(self.wall_line_list[i].angle - math.pi / 2),
                                    wall_line_list[i].angle))

        ## make marker
        marker_array = MarkerArray()
        for i in range(self.wall_num):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "guide_line"
            marker.id = i
            marker.type = marker.LINE_STRIP
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.color.a = 0.5
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.points.append(Point(guide_line_list[i].x1, guide_line_list[i].y1,0))
            marker.points.append(Point(guide_line_list[i].x2, guide_line_list[i].y2,0))
            marker_array.markers.append(marker)

        return marker_array

    def main(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            ## check if yaml is updated
            new_wall_list = []
            self.wall_num = 0
            self.wall_yaml = self.load_data()
            self.make_list(new_wall_list)
            if new_wall_list != self.wall_line_list:
                self.wall_line_list = new_wall_list
                print('wall_list is updated')
                print('wall_listsize = ', self.wall_num)
                # print('wall_list = ', self.wall_line_list)
            if self.wall_num == 0:
                print('wall_list is empty')
                continue

            ## make marker
            wall_marker_array = self.make_wall_marker(self.wall_line_list)
            guide_marker_array = self.make_guide_line(self.wall_line_list)

            ## publish
            self.pub_wall.publish(wall_marker_array)
            self.pub_guide.publish(guide_marker_array)

            rate.sleep()

if __name__ == '__main__':
    make_guide_line = MakeGuideLine()
    make_guide_line.main()
