#!/usr/bin/env python

import rospy
import yaml
import math
import copy

from geometry_msgs.msg import PointStamped, Point, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import OccupancyGrid

from route_maker.msg import Wall, WallArray

class WallRecoder:
    def __init__(self):
        rospy.init_node('wall_recorder')
        print('=== Wall Recorder ===')

        ## check param
        self.hz = rospy.get_param('~hz', 10)
        self.points = []
        self.id = 0
        self.wall = Wall()
        self.wall_array = WallArray()
        self.walled_map = OccupancyGrid()
        self.get_map = False

        ## subscriber
        self.sub_point = rospy.Subscriber('clicked_point', PointStamped, self.point_callback)
        self.sub_del_point = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.del_point_callback)
        self.sub_map = rospy.Subscriber('map', OccupancyGrid, self.map_callback)

        ## publisher
        self.pub_wall = rospy.Publisher('wall', WallArray, queue_size=1)
        self.pub_map = rospy.Publisher('walled_map', OccupancyGrid, queue_size=1)
        ## visualization
        self.pub_clicked_point = rospy.Publisher('clicked_point_marker', Marker, queue_size=1)


    def map_callback(self,msg):
        self.map = msg
        self.walled_map.header = self.map.header
        self.walled_map.info = self.map.info
        self.walled_map.data = list(self.map.data)
        self.get_map = True
        print('get map')

    def point_callback(self,msg):
        x = round(msg.point.x, 3)
        y = round(msg.point.y, 3)
        z = round(msg.point.z, 3)
        clicked = Point(x,y,z)
        self.points.append(clicked)

        if not self.get_map:
            print('no map received')
            return

        if len(self.points) and len(self.points) % 2 == 1:
            self.show_clicked_point(clicked)
            print('Click next point')

        if len(self.points) and len(self.points) % 2 == 0:
            self.show_clicked_point(clicked)
            self.wall.start = self.points[2*self.id]
            self.wall.end = clicked
            angle = math.atan2(self.wall.end.y - self.wall.start.y, self.wall.end.x - self.wall.start.x)
            self.wall.angle = round(angle, 3)
            length = math.sqrt((self.wall.end.x - self.wall.start.x)**2 + (self.wall.end.y - self.wall.start.y)**2)
            self.wall.length = round(length, 3)
            self.wall_array.walls.append(copy.deepcopy(self.wall))
            self.draw_on_map(self.wall, 100)
            print('pub wall')

            self.id += 1

    def del_point_callback(self,msg):
        if len(self.points) %2 == 1:
            print('error! cannot delete wall now')
            return
        if len(self.points) == 0:
            print('no wall to delete')
            return
        x = round(msg.pose.position.x, 3)
        y = round(msg.pose.position.y, 3)
        z = round(msg.pose.position.z, 3)
        clicked = Point(x,y,z)
        delete_wall = self.check_nearest_wall(clicked)
        if delete_wall == None:
            print ('no wall to delete')
            return
        self.draw_on_map(delete_wall ,0)
        self.wall_array.walls.remove(delete_wall)

    def check_nearest_wall(self,clicked):
        min_dist = 5
        nearest_wall = Wall()
        for wall in self.wall_array.walls:
            a,b = self.get_line(wall.start.x, wall.start.y, wall.end.x, wall.end.y)
            dist = self.get_distance(clicked,a,b)
            if dist < min_dist:
                min_dist = dist
                nearest_wall = wall
                print(dist)
        if dist != 5:
            return nearest_wall
        else:
            return None

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

    def draw_on_map(self,wall, value):
        start_x, start_y = self.map_to_grid(wall.start.x, wall.start.y)
        end_x, end_y = self.map_to_grid(wall.end.x, wall.end.y)
        a,b = self.get_line(start_x, start_y, end_x, end_y)
        min_x = min(start_x, end_x)
        max_x = max(start_x, end_x)

        x = min_x
        while x <= max_x:
            y = int(a*x + b)
            index = int(self.map.info.width * y + x)
            self.walled_map.data[index] = value
            x += self.map.info.resolution

        # self.pub_map.publish(self.walled_map)

    def map_to_grid(self,x,y):
        grid_x = int((x - self.map.info.origin.position.x) / self.map.info.resolution)
        grid_y = int((y - self.map.info.origin.position.y) / self.map.info.resolution)
        return grid_x, grid_y

    def get_line(self,x1,y1,x2,y2):
        if x1 == x2:
            a = 0
        else:
            a = (y1 - y2) / (x1 - x2)
        b = y1 - a*x1
        return a,b

    def get_distance(self,point,a,b):
        dist = abs(a*point.x - point.y + b) / math.sqrt(a**2 + 1)
        return dist

    def main(self):
        rate = rospy.Rate(self.hz)
        while not rospy.is_shutdown():
            self.pub_map.publish(self.walled_map)
            rate.sleep()

if __name__ == '__main__':
    wall_recoder = WallRecoder()
    wall_recoder.main()





