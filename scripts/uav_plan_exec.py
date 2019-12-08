#!/usr/bin/env python

# 3rd Party Packages
import argparse

# ROS Packages
import rospy
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.srv import DispatchService
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceRequest
from std_srvs.srv import Empty
from uav_rosplan.interface import UAVActionInterface
from uav_rosplan.rosplan_interface import UAVInterface


class UAVExec(object):
    def __init__(self, mavlink_conn, update_frequency=4.):
        self.goal_state = list()
        self.uav = UAVInterface(name='hector',
                                mavlink_conn=mavlink_conn,
                                uav=UAVActionInterface())
        # Service proxies
        rospy.loginfo('Waiting for rosplan services...')
        rospy.wait_for_service(
            '/rosplan_problem_interface/problem_generation_server')
        self._problem_proxy = rospy.ServiceProxy(
            '/rosplan_problem_interface/problem_generation_server', Empty)
        rospy.wait_for_service('/rosplan_planner_interface/planning_server')
        self._planner_proxy = rospy.ServiceProxy(
            '/rosplan_planner_interface/planning_server', Empty)
        rospy.wait_for_service('/rosplan_parsing_interface/parse_plan')
        self._parser_proxy = rospy.ServiceProxy(
            '/rosplan_parsing_interface/parse_plan', Empty)
        rospy.wait_for_service('/rosplan_plan_dispatcher/dispatch_plan')
        self._dispatch_proxy = rospy.ServiceProxy(
            '/rosplan_plan_dispatcher/dispatch_plan', DispatchService)
        self._cancel_plan_proxy = rospy.ServiceProxy(
            '/rosplan_plan_dispatcher/cancel_dispatch', Empty)
        # Service
        rospy.Service('%s/resume_plan' % rospy.get_name(), Empty,
                      self.resume_plan)
        # Auto call functions
        self._rate = rospy.Rate(update_frequency)
        rospy.Timer(self._rate.sleep_dur, self.lowbat_return)

    def resume_plan(self, req):
        """
        Function to flag down external intervention, and replan
        """
        self.uav.resume_plan()
        rospy.Timer(self._rate.sleep_dur, self.execute, oneshot=True)
        return list()

    def execute(self, event=True):
        """
        Execute plan using ROSPlan
        """
        rospy.loginfo('Generating mission plan ...')
        self._problem_proxy()
        self._rate.sleep()
        rospy.loginfo('Planning ...')
        self._planner_proxy()
        self._rate.sleep()
        rospy.loginfo('Execute mission plan ...')
        self._parser_proxy()
        self._rate.sleep()
        response = self._dispatch_proxy()
        if not response.goal_achieved:
            rospy.loginfo('Mission Failed')
        else:
            rospy.loginfo('Mission Succeed')
        return response.goal_achieved

    def lowbat_return(self, event=True):
        """
        UAV return to launch due to low battery voltage
        """
        if self.uav.lowbat:
            self._rate.sleep()
            update_types = [
                KnowledgeUpdateServiceRequest.REMOVE_GOAL
                for _ in self.goal_state[0]
            ]
            self.uav.update_predicates(self.goal_state[0], self.goal_state[1],
                                       update_types)
            self._rate.sleep()
            pred_names = ['landed', 'uav_at']
            params = [
                [KeyValue('v', 'hector')],
                [KeyValue('v', 'hector'),
                 KeyValue('wp', 'wp0')],
            ]
            self.goal_state = (pred_names, params)
            update_types = [
                KnowledgeUpdateServiceRequest.ADD_GOAL,
                KnowledgeUpdateServiceRequest.ADD_GOAL
            ]
            self.uav.update_predicates(pred_names, params, update_types)
            self._rate.sleep()
            self.execute()

    def fly_test_mission(self):
        """
        UAV mission visiting all waypoints and coming back to launch pod
        """
        pred_names = [
            'visited', 'visited', 'visited', 'visited', 'visited', 'landed'
        ]
        params = [[KeyValue('wp', 'wp1')], [KeyValue('wp', 'wp2')],
                  [KeyValue('wp', 'wp3')], [KeyValue('wp', 'wp4')],
                  [KeyValue('wp', 'wp5')], [KeyValue('v', 'hector')]]
        self.goal_state = (pred_names, params)
        update_types = [
            KnowledgeUpdateServiceRequest.ADD_GOAL,
            KnowledgeUpdateServiceRequest.ADD_GOAL,
            KnowledgeUpdateServiceRequest.ADD_GOAL,
            KnowledgeUpdateServiceRequest.ADD_GOAL,
            KnowledgeUpdateServiceRequest.ADD_GOAL,
            KnowledgeUpdateServiceRequest.ADD_GOAL
        ]
        succeed = self.uav.update_predicates(pred_names, params, update_types)
        self._rate.sleep()
        return succeed


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
    uav.fly_test_mission()
    uav.execute()
    rospy.spin()
