#!/usr/bin/env python3

import rospy
import yaml

from brushee_navigation_msgs.msg import BrusheeCleaningPath , Node, Edge, NodeEdge

class node:
    def __init__(self, node_id, n_type, pose, direction):
        self.node_id = node_id
        self.type = n_type
        self.pose = pose
        self.direction = direction

class edge:
    def __init__(self, p1, p2, command, skippable):
        self.start = p1
        self.end = p2
        self.command = command
        self.skippable = skippable

class MsgToYaml:
    def __init__(self):
        rospy.init_node('msg_to_yaml')
        print('=== Msg To Yaml ===')

        self.get_msg = False

        self.nodes = []
        self.edges = []

        self.path_dict = {}
        self.path_dict['PATH_FRAME'] = []
        self.path_dict['PATH_FRAME'].append('map')
        self.path_dict['OCC_MAP_NAME'] = []
        self.path_dict['OCC_MAP_NAME'].append('map')
        self.path_dict['MAP_DIRECTION'] = []
        self.path_dict['MAP_DIRECTION'].append('0.0')
        self.path_dict['NODE'] = []
        self.path_dict['EDGE'] = []

        self.yaml_file = rospy.get_param('~yaml_file', 'path.yaml')
        self.msg_topic = rospy.get_param('~msg_topic', '/new_node_edge')

        rospy.Subscriber(self.msg_topic, NodeEdge, self.msg_callback)


    def msg_callback(self, msg):
        self.nodes.clear()
        self.edges.clear()
        self.path_dict['NODE'].clear()
        self.path_dict['EDGE'].clear()

        self.msg = msg
        for n in msg.nodes:
            self.nodes.append(node(n.id, n.type, n.pose, n.direction))
        for e in msg.edges:
            self.edges.append(edge(e.start_node_id, e.end_node_id, e.command, e.skippable))
        # print('nodes:', self.nodes)
        # print('edges:', self.edges)
        self.add_to_dict(self.nodes, self.edges, self.path_dict)
        self.write_yaml()
        print('Get msg')

    def add_to_dict(self,node,edge,path_dict):
        for n in node:
            path_dict['NODE'].append({'id':n.id, 'type':n.type, 'x':n.pose.x, 'y':n.pose.y, 'direction':n.direction})

        for e in edge:
            path_dict['EDGE'].append({'command':e.command, 'start_node_id':e.start, 'end_node_id':e.end, 'skippable':e.skippable})

    def write_yaml(self):
        with open(self.yaml_file, 'w') as f:
            yaml.dump(self.path_dict, f)
        print('Write to yaml file:', self.yaml_file)

    def process(self):
        rospy.spin()


if __name__ == '__main__':
    msg_to_yaml = MsgToYaml()
    msg_to_yaml.process()


