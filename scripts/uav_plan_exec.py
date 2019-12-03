#!/usr/bin/env python

# 3rd Party Packages
import argparse

# ROS Packages
import rospy
from diagnostic_msgs.msg import KeyValue
from uav_rosplan.interface import UAVActionInterface
from uav_rosplan.rosplan_interface import UAVInterface
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest


class UAVExec(object):
    def __init__(self, mavlink_conn):
        self.uav = UAVInterface(name='hector',
                                mavlink_conn=mavlink_conn,
                                uav=UAVActionInterface())

    def mission_goal(self):
        pred_names = [
            'visited', 'visited', 'visited', 'visited', 'visited', 'landed'
        ]
        params = [[KeyValue('wp', 'wp1')], [KeyValue('wp', 'wp2')],
                  [KeyValue('wp', 'wp3')], [KeyValue('wp', 'wp4')],
                  [KeyValue('wp', 'wp5')], [KeyValue('v', 'hector')]]
        update_types = [
            KnowledgeUpdateServiceRequest.ADD_GOAL,
            KnowledgeUpdateServiceRequest.ADD_GOAL,
            KnowledgeUpdateServiceRequest.ADD_GOAL,
            KnowledgeUpdateServiceRequest.ADD_GOAL,
            KnowledgeUpdateServiceRequest.ADD_GOAL,
            KnowledgeUpdateServiceRequest.ADD_GOAL
        ]
        return self.uav.update_predicates(pred_names, params, update_types)


if __name__ == '__main__':
    rospy.init_node('uav_executive')
    parser_arg = argparse.ArgumentParser(prog=rospy.get_name())
    parser_arg.add_argument('mavlink_conn', help='Mavlink Connection')
    parser_arg.add_argument("-f",
                            dest='wp_file',
                            default='wp_config.yaml',
                            help='waypoint\'s file name')
    parser_arg.add_argument('-wp',
                            dest='waypoint',
                            default='0',
                            help='Go to specific waypoint')
    args = parser_arg.parse_args()
    rospy.set_param('~wp_file', args.wp_file)
    uav = UAVExec(args.mavlink_conn)
    uav.mission_goal()
    rospy.spin()
