#!/usr/bin/env python

# 3rd Party Packages
from threading import Lock

# ROS Packages
import rospy
from diagnostic_msgs.msg import KeyValue
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_knowledge_msgs.srv import (GetDomainOperatorDetailsService,
                                        GetDomainPredicateDetailsService,
                                        KnowledgeUpdateService,
                                        KnowledgeUpdateServiceRequest)
from uav_rosplan.interface import UAVActionInterface
from uav_rosplan.preflightcheck import PreFlightCheck


class UAVInterface(object):

    mutex = Lock()

    def __init__(self,
                 name='UAV1',
                 mavlink_conn='',
                 uav=None,
                 update_frequency=10.):
        """
        A Class that interfaces ROSPlan and MAVROS for executing UAV actions
        """
        self.uav = uav
        self.name = name
        self.uav_landed = False
        self.uav_wp = -1
        self.guided = False
        self.armed = False
        self.lowbat = False
        self.mavlink_conn = mavlink_conn
        if uav is None:
            self.uav = UAVActionInterface()
        # Service proxies
        rospy.loginfo('Waiting for service /rosplan_knowledge_base/update ...')
        rospy.wait_for_service('/rosplan_knowledge_base/update')
        self._knowledge_update_proxy = rospy.ServiceProxy(
            '/rosplan_knowledge_base/update', KnowledgeUpdateService)
        rospy.loginfo(
            'Waiting for /rosplan_knowledge_base/domain/predicate_details ...')
        rospy.wait_for_service(
            '/rosplan_knowledge_base/domain/predicate_details')
        self._predicate_proxy = rospy.ServiceProxy(
            '/rosplan_knowledge_base/domain/predicate_details',
            GetDomainPredicateDetailsService)
        rospy.wait_for_service(
            '/rosplan_knowledge_base/domain/operator_details')
        self._operator_proxy = rospy.ServiceProxy(
            '/rosplan_knowledge_base/domain/operator_details',
            GetDomainOperatorDetailsService)
        # Subscribers
        rospy.Subscriber('/rosplan_plan_dispatcher/action_dispatch',
                         ActionDispatch,
                         self._dispatch_cb,
                         queue_size=10)
        # Publishers
        self._feedback_publisher = rospy.Publisher(
            '/rosplan_plan_dispatcher/action_feedback',
            ActionFeedback,
            queue_size=10)

        self._rate = rospy.Rate(update_frequency)
        self.instances_set = self.set_instances()
        self.init_set = False
        if not self.instances_set:
            rospy.logerr('%s\'s instances in PDDL can\'t be set!' % self.name)
        else:
            self.init_set = self.set_init()
        if not self.init_set:
            rospy.logerr('%s\'s initial state can\'t be set!' % self.name)
        # Auto call functions
        rospy.Timer(self._rate.sleep_dur, self.knowledge_update)

    def _apply_operator_effect(self, op_name, dispatch_params):
        """
        Add / remove knowledge based on operator effect and parameters
        from dispatched action
        """
        predicate_names = list()
        parameters = list()
        update_types = list()
        response = self._operator_proxy(op_name)
        for predicate in response.op.at_start_add_effects:
            predicate_names.append(predicate.name)
            params = list()
            for typed_param in predicate.typed_parameters:
                for param in dispatch_params:
                    if typed_param.key == param.key:
                        params.append(param)
                        break
            parameters.append(params)
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        for predicate in response.op.at_start_del_effects:
            predicate_names.append(predicate.name)
            params = list()
            for typed_param in predicate.typed_parameters:
                for param in dispatch_params:
                    if typed_param.key == param.key:
                        params.append(param)
                        break
            parameters.append(params)
            update_types.append(KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE)
        succeed = self.update_predicates(predicate_names, parameters,
                                         update_types)
        return succeed

    def _dispatch_cb(self, msg):
        """
        Function for action_dispatch callback
        """
        rospy.loginfo('%s: action received' % self.name)
        # parse action message
        if msg.name == 'preflightcheck':
            self._action(msg, self.preflightcheck)
        elif msg.name == 'guide_mode':
            self._action(msg, self.uav.guided_mode)
        elif msg.name == 'request_arm':
            self._action(msg, self.uav.request_arm)
        elif msg.name == 'takeoff':
            self._action(msg, self.uav.takeoff,
                         [rospy.get_param('~takeoff_altitude', 10.)])
        elif msg.name == 'goto_waypoint':
            self._action(msg, self.goto_waypoint, [msg.parameters])
        elif msg.name == 'rtl':
            self._action(msg, self.uav.return_to_land, [False])
        elif msg.name == 'lowbat_return':
            self._action(msg, self.uav.return_to_land, [False])

    def _action(self, action_dispatch, action_func, action_params=list()):
        """
        Template uav action for generic uav action
        such as request_arm, takeoff etc
        """
        self.publish_feedback(action_dispatch.action_id, 'action enabled')
        start_time = rospy.Time(action_dispatch.dispatch_time)
        duration = rospy.Duration(action_dispatch.duration)
        self._rate.sleep()
        rospy.loginfo('Dispatching %s action at %s with duration %s ...' %
                      (action_dispatch.name, str(
                          start_time.secs), str(duration.to_sec())))
        if action_func(*action_params) == self.uav.ACTION_SUCCESS:
            if self._apply_operator_effect(action_dispatch.name,
                                           action_dispatch.parameters):
                self.publish_feedback(action_dispatch.action_id,
                                      'action achieved')
            else:
                self.publish_feedback(action_dispatch.action_id,
                                      'action failed')
        else:
            self.publish_feedback(action_dispatch.action_id, 'action failed')

    def resume_plan(self):
        """
        Function to flag down external intervention
        """
        self.uav.previous_mode = self.uav.current_mode
        self.uav.current_mode = self.uav.state.mode
        self.uav.external_intervened = False

    def knowledge_update(self, event):
        """
        Add or remove facts related to this UAV at ROSPlan knowledge base
        """
        pred_names = list()
        params = list()
        update_types = list()
        # landing status update
        if self.uav.landed and not self.uav_landed:
            update_type = [
                KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE,
                KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE
            ]
        elif not self.uav.landed and self.uav_landed:
            update_type = [
                KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE,
                KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE
            ]
        if self.uav.landed != self.uav_landed:
            pred_names.extend(['landed', 'airborne'])
            params.extend([[KeyValue('v', self.name)],
                           [KeyValue('v', self.name)]])
            update_types.extend(update_type)
            self.uav_landed = self.uav.landed
        # uav position in waypoints update
        wp_seq = self.uav._current_wp
        # only update when self.uav_wp != wp_seq
        if wp_seq != -1 and self.uav_wp != wp_seq:
            # add current wp that uav resides
            pred_names.append('uav_at')
            params.append(
                [KeyValue('v', self.name),
                 KeyValue('wp', 'wp%d' % wp_seq)])
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
            # remove previous wp that uav resided
            if self.uav_wp != -1:
                pred_names.append('uav_at')
                params.append([
                    KeyValue('v', self.name),
                    KeyValue('wp', 'wp%d' % self.uav_wp)
                ])
                update_types.append(
                    KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE)
            # update visited state
            pred_names.append('visited')
            params.append([KeyValue('wp', 'wp%d' % wp_seq)])
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
            self.uav_wp = wp_seq
        # guided status update
        update_guided = False
        if self.uav.state.mode == 'GUIDED' and not self.guided:
            update_type = KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE
            update_guided = True
        elif self.uav.state.mode != 'GUIDED':
            update_type = KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE
            update_guided = True
        if update_guided:
            pred_names.append('guided')
            params.append([KeyValue('v', self.name)])
            update_types.append(update_type)
            self.guided = (self.uav.state.mode == 'GUIDED')
        # arm status update
        if self.uav.state.armed and not self.armed:
            update_type = KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE
        elif not self.uav.state.armed and self.armed:
            update_type = KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE
        if self.uav.state.armed != self.armed:
            pred_names.append('armed')
            params.append([KeyValue('v', self.name)])
            update_types.append(update_type)
            self.armed = self.uav.state.armed
        # battery status update
        if self.uav.low_battery and not self.lowbat:
            update_type = KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE
        elif not self.uav.low_battery and self.lowbat:
            update_type = KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE
        if self.uav.low_battery != self.lowbat:
            pred_names.append('lowbat')
            params.append([KeyValue('v', self.name)])
            update_types.append(update_type)
            self.lowbat = self.uav.low_battery
        # update state when necessary
        if pred_names != list():
            self.update_predicates(pred_names, params, update_types)

    def set_init(self):
        """
        Set extra facts in initial states
        """
        pred_names = ['home']
        params = [[KeyValue('wp', 'wp0')]]
        update_types = [KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE]
        succeed = self.update_predicates(pred_names, params, update_types)
        return succeed

    def set_instances(self):
        """
        Set initial instances for this UAV to ROSPlan
        """
        ins_types = list()
        ins_names = list()
        update_types = list()
        for idx in range(len(self.uav.waypoints)):
            ins_types.append('waypoint')
            ins_names.append('wp' + str(idx))
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        ins_types.append('uav')
        ins_names.append(self.name)
        update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        return self.update_instances(ins_types, ins_names, update_types)

    def update_instances(self, ins_types, ins_names, update_types):
        """
        Add / remove instances
        """
        success = True
        for idx, ins_type in enumerate(ins_types):
            req = KnowledgeUpdateServiceRequest()
            req.knowledge.knowledge_type = KnowledgeItem.INSTANCE
            req.knowledge.instance_type = ins_type
            req.knowledge.instance_name = ins_names[idx]
            req.update_type = update_types[idx]
            success = success and self._knowledge_update_proxy(req).success
        return success

    def update_predicates(self, pred_names, parameters, update_types):
        """
        Add / remove predicate facts or goals
        """
        self.mutex.acquire()
        success = True
        for idx, pred_name in enumerate(pred_names):
            req = KnowledgeUpdateServiceRequest()
            req.knowledge.knowledge_type = KnowledgeItem.FACT
            req.knowledge.attribute_name = pred_name
            req.knowledge.values.extend(parameters[idx])
            req.update_type = update_types[idx]
            success = success and self._knowledge_update_proxy(req).success
        self.mutex.release()
        return success

    def publish_feedback(self, action_id, fbstatus):
        """
        Function to publish action feedback to action_feedback topic
        """
        feedback = ActionFeedback()
        feedback.action_id = action_id
        feedback.status = fbstatus
        self._feedback_publisher.publish(feedback)

    def preflightcheck(self, duration=rospy.Duration(600, 0)):
        """
        Preflight check action for UAV
        """
        start = rospy.Time.now()
        # preflight process
        preflightcheck = PreFlightCheck(self.mavlink_conn, self.uav)
        try:
            init_batt = float(preflightcheck._init_batt.get())
            min_batt = float(preflightcheck._low_batt.get())
            self.uav.set_battery(init_batt, min_batt)
            response = self.uav.ACTION_SUCCESS
        except ValueError:
            rospy.logwarn('Pre-flight check doesn\'t have correct inputs!')
            response = self.uav.ACTION_FAIL
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        return response

    def goto_waypoint(self, dispatch_params, duration=rospy.Duration(60, 0)):
        """
        Go to waypoint action for UAV
        """
        waypoint = -1
        for param in dispatch_params:
            if param.key == 'to':
                waypoint = int(param.value[2:])
                break
        response = self.uav.goto(
            waypoint, duration) if waypoint != -1 else self.uav.ACTION_FAIL
        return response
