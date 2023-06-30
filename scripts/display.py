#!/usr/bin/env python3

## Display areas

import rospy
import yaml

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from route_maker.msg import Area, AreaArray, Wall, WallArray

class Display:
    def __init__(self):
        rospy.init_node('display', anonymous=True)
        print('=== Display ===')

        ## check param
        if rospy.has_param('~FILE_NAME'):
            self.area_file = rospy.get_param('~FILE_NAME')
            print('file_name: ' + self.area_file)

        self.hz = rospy.get_param('~HZ', 10)

        self.area_array = AreaArray()
        self.wall_array = WallArray()
        self.start_end_points = []
        self.is_empty = True

        ## publisher
        self.pub_area_marker = rospy.Publisher('area_markers', MarkerArray, queue_size=1)
        self.pub_wall_marker = rospy.Publisher('wall_markers', MarkerArray, queue_size=1)
        self.pub_start_end_marker = rospy.Publisher('start_end_markers', MarkerArray, queue_size=1)

    def load_data(self):
        with open(self.area_file, 'r') as f:
            area_yaml = yaml.safe_load(f)
        return area_yaml

    def make_area_list(self, area_yaml, area_array):
        if len(area_yaml['AREA']) > 0:
            self.is_empty = False

        for area in area_yaml['AREA']:
            area_array.areas.append(area)

    def make_lists(self, area_array, wall_array, start_end_points):
        for area in area_array.areas:
            wall_array.walls.append(area['wall'])
            start_end_points.append(area['start'])
            start_end_points.append(area['end'])

    def show_areas(self, area_array):
        marker_array = MarkerArray()
        for area in area_array.areas:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "area"
            marker.id = area['id']
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0

            p1 = Point()
            p1.x = area['p1']['x']
            p1.y = area['p1']['y']
            p1.z = 0.0
            p2 = Point()
            p2.x = area['p2']['x']
            p2.y = area['p2']['y']
            p2.z = 0.0
            p3 = Point()
            p3.x = area['p3']['x']
            p3.y = area['p3']['y']
            p3.z = 0.0
            p4 = Point()
            p4.x = area['p4']['x']
            p4.y = area['p4']['y']
            p4.z = 0.0

            marker.points.append(p1)
            marker.points.append(p2)

            marker.points.append(p2)
            marker.points.append(p3)

            marker.points.append(p3)
            marker.points.append(p4)

            marker.points.append(p4)
            marker.points.append(p1)

            marker_array.markers.append(marker)

        self.pub_area_marker.publish(marker_array)

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
            start  = Point()
            start.x = wall['start']['x']
            start.y = wall['start']['y']
            start.z = 0.0
            end  = Point()
            end.x = wall['end']['x']
            end.y = wall['end']['y']
            end.z = 0.0
            marker.points.append(start)
            marker.points.append(end)
            marker_array.markers.append(marker)
            i += 1

        self.pub_wall_marker.publish(marker_array)

    def show_start_end(self, start_end_points):
        ## yellow and pink dict
        color_dict = {0:{'r':1.0,'g':1.0,'b':0.0},1:{'r':1.0,'g':0.0,'b':1.0}}
        marker_array = MarkerArray()
        for i in range(len(start_end_points)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "start_end"
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
            marker.pose.position.x = start_end_points[i]['x']
            marker.pose.position.y = start_end_points[i]['y']
            marker.pose.position.z = 0.0
            marker_array.markers.append(marker)

        self.pub_start_end_marker.publish(marker_array)

    def publish(self):
        self.show_areas(self.area_array)
        self.show_walls(self.wall_array)
        self.show_start_end(self.start_end_points)

    def main(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            ## check if yaml file is updated
            new_area_array = AreaArray()
            self.is_empty = True
            self.area_yaml = self.load_data()
            self.make_area_list(self.area_yaml, new_area_array)
            if self.is_empty:
                print('No area data')
                continue
            if new_area_array != self.area_array:
                self.area_array = new_area_array
                self.make_lists(self.area_array, self.wall_array, self.start_end_points)
                print('area_array updated')
            ## publish
            self.publish()
            rate.sleep()

if __name__ == '__main__':
    display = Display()
    display.main()
