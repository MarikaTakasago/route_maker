#!/usr/bin/env python

import rospy
import yaml

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

class WallRecorder:
    def __init__(self):
        rospy.init_node('wall_recorder')
        print('=== Wall Recorder ===')



