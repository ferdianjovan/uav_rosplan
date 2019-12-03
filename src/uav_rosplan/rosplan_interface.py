#!/usr/bin/env python

import rospy
from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.msg import KnowledgeItem
from rosplan_dispatch_msgs.msg import ActionDispatch, ActionFeedback
from rosplan_knowledge_msgs.srv import (GetDomainPredicateDetailsService,
                                        KnowledgeUpdateService,
                                        KnowledgeUpdateServiceRequest,
                                        GetDomainOperatorDetailsService)
from uav_rosplan.interface import UAVActionInterface
from uav_rosplan.preflightcheck import PreFlightCheck


class UAVInterface(object):
    def __init__(self,
                 name='UAV1',
                 mavlink_conn='',
                 uav=None,
                 update_frequency=10.):
        """
        A Class that interfaces ROSPlan and MAVROS for executing actions
        """
        self.uav = uav
        self.name = name
        self.uav_landed = False
        self.uav_wp = -1
        self.guided = False
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
            rospy.logerr('Object instances for PDDL problem can\'t be set!')
        else:
            self.init_set = self.set_init()
        if not self.init_set:
            rospy.logerr('Initial state for PDDL problem can\'t be set!')
        # Auto call functions
        rospy.Timer(rospy.Duration(1. / update_frequency),
                    self.knowledge_update)

    def _apply_operator_effect(self, op_name, dispatch_params):
        """
        Add / remove knowledge based on operator effect and parameters
        from dispatched action
        """
        predicate_names = list()
        parameters = list()
        update_types = list()
        response = self._operator_proxy(op_name)
        for predicate in response.op.formula.at_start_add_effects:
            predicate_names.append(predicate.name)
            params = list()
            for typed_param in predicate.typed_parameters:
                for param in dispatch_params:
                    if typed_param.key == param.key:
                        params.append(param)
                        break
            parameters.append(params)
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
        for predicate in response.op.formula.at_start_del_effects:
            predicate_names.append(predicate.name)
            params = list()
            for typed_param in predicate.typed_parameters:
                for param in dispatch_params:
                    if typed_param.key == param.key:
                        params.append(param)
                        break
            parameters.append(params)
            update_types.append(KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE)
        return self.update_predicates(predicate_names, parameters,
                                      update_types)

    # def _action_template(self,

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
            pred_names.append(['landed', 'airborne'])
            params.append([KeyValue('v', self.name)],
                          [KeyValue('v', self.name)])
            update_types.extend(update_type)
            self.uav_landed = self.uav.landed
        # uav position in waypoints update
        wp_seq = -1
        if self.uav.wp_reached == -1:
            latitude_cond = abs(self.uav.global_pose.latitude -
                                self.uav.home.geo.latitude) < 1e-05
            longitude_cond = abs(self.uav.global_pose.longitude -
                                 self.uav.home.geo.longitude) < 1e-05
            altitude_cond = abs(self.uav.global_pose.altitude -
                                self.uav.home.geo.altitude) < 0.1
            within_perimeter = (latitude_cond and longitude_cond)
            within_perimeter = (within_perimeter and altitude_cond)
            wp_seq = 0 if within_perimeter else -1
        else:
            wp_seq = self.uav.wp_reached
        # wp_seq != -1 means it is in any wp
        # only update when self.uav_wp != wp_seq
        if wp_seq != -1 and self.uav_wp != wp_seq:
            # add current wp that uav resides
            pred_names.append('uav_at')
            params.append(
                [KeyValue('v', self.name),
                 KeyValue('wp', 'wp%d' % wp_seq)])
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
            # remove previous wp that uav resided
            pred_names.append('uav_at')
            params.append([
                KeyValue('v', self.name),
                KeyValue('wp', 'wp%d' % self.uav_wp)
            ])
            update_types.append(KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE)
            # update visited state
            pred_names.append('visited')
            params.append([KeyValue('wp', 'wp%d' % wp_seq)])
            update_types.append(KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE)
            self.uav_wp = wp_seq
        # guided status update
        if self.uav.state.guided and not self.guided:
            update_type = KnowledgeUpdateServiceRequest.ADD_KNOWLEDGE,
        elif not self.uav.state.guided and self.guided:
            update_type = KnowledgeUpdateServiceRequest.REMOVE_KNOWLEDGE,
        if self.uav.state.guided != self.guided:
            pred_names.append(['guided'])
            params.append([KeyValue('v', self.name)])
            update_types.append(update_type)
            self.guided = self.uav.state.guided
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
        return self.update_predicates(pred_names, params, update_types)

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
        success = True
        for idx, pred_name in enumerate(pred_names):
            req = KnowledgeUpdateServiceRequest()
            req.knowledge.knowledge_type = KnowledgeItem.FACT
            req.knowledge.attribute_name = pred_name
            req.knowledge.values.extend(parameters[idx])
            req.update_type = update_types[idx]
            success = success and self._knowledge_update_proxy(req).success
        return success

    def _dispatch_cb(self, msg):
        """
        Function for action_dispatch callback
        """
        rospy.loginfo('%s: action received' % self.name)
        # parse action message
        if msg.name == 'preflightcheck':
            self.preflightcheck(msg)
        elif msg.name == 'guide_mode':
            self.guide_mode(msg)
        elif msg.name == 'request_arm':
            self.request_arm(msg)
        elif msg.name == 'takeoff':
            self.takeoff(msg)
        elif msg.name == 'goto_waypoint':
            self.goto_waypoint(msg)
        elif msg.name == 'rtl':
            self.rtl(msg)

    def publish_feedback(self, action_id, fbstatus):
        """
        Function to publish action feedback to action_feedback topic
        """
        feedback = ActionFeedback()
        feedback.action_id = action_id
        feedback.status = fbstatus
        self._feedback_publisher.publish(feedback)

    def _match_predicate_params(self, pred_name, params):
        """
        Function to find matching KeyValue for predicates
        """
        matched = list()
        response = self._predicate_proxy(pred_name)
        for typed_param in response.predicate.typed_parameters:
            for param in params:
                if typed_param.key == param.key:
                    matched.append(param)
                    break
        return matched

    def preflightcheck(self, action_dispatch):
        """
        Pre-flight check action interfacing with ROSPlan
        """
        self.publish_feedback(action_dispatch.action_id, 'action enabled')
        start_time = rospy.Time(action_dispatch.dispatch_time)
        duration = rospy.Duration(action_dispatch.duration)
        self._rate.sleep()
        rospy.loginfo('Dispatching setting mode to %s action ...' %
                      action_dispatch.name)
        rospy.loginfo('Dispatch action at %s with duration %s' %
                      (str(start_time.secs), str(duration.to_sec())))
        # preflight process
        preflightcheck = PreFlightCheck(self.mavlink_conn, self.uav)
        try:
            init_batt = float(preflightcheck._init_batt.get())
            min_batt = float(preflightcheck._low_batt.get())
            self.uav.set_battery(init_batt, min_batt)
            if self._apply_operator_effect(action_dispatch.name,
                                           action_dispatch.parameters):
                self.publish_feedback(action_dispatch.action_id,
                                      'action achieved')
            else:
                self.publish_feedback(action_dispatch.action_id,
                                      'action failed')
        except ValueError:
            rospy.logwarn('Pre-flight check doesn\'t have correct inputs!')
            self.publish_feedback(action_dispatch.action_id, 'action failed')

    def request_arm(self, action_dispatch):
        """
        Request arm action interfacing with ROSPlan
        """
        self.publish_feedback(action_dispatch.action_id, 'action enabled')
        start_time = rospy.Time(action_dispatch.dispatch_time)
        duration = rospy.Duration(action_dispatch.duration)
        self._rate.sleep()
        rospy.loginfo('Dispatching setting mode to %s action ...' %
                      action_dispatch.name)
        rospy.loginfo('Dispatch action at %s with duration %s' %
                      (str(start_time.secs), str(duration.to_sec())))
        rospy.loginfo('Waiting for the UAV to be ARMED ...')
        while not self.uav.state.armed:
            self._rate.sleep()
        if self.uav.state.armed:
            if self._apply_operator_effect(action_dispatch.name,
                                           action_dispatch.parameters):
                self.publish_feedback(action_dispatch.action_id,
                                      'action achieved')
            else:
                self.publish_feedback(action_dispatch.action_id,
                                      'action failed')
        else:
            self.publish_feedback(action_dispatch.action_id, 'action failed')

    def guide_mode(self, action_dispatch):
        """
        Setting uav to guide mode action interfacing with ROSPlan
        """
        self.publish_feedback(action_dispatch.action_id, 'action enabled')
        start_time = rospy.Time(action_dispatch.dispatch_time)
        duration = rospy.Duration(action_dispatch.duration)
        self._rate.sleep()
        rospy.loginfo('Dispatching setting mode to %s action ...' %
                      action_dispatch.name)
        rospy.loginfo('Dispatch action at %s with duration %s' %
                      (str(start_time.secs), str(duration.to_sec())))
        action_succeed = self.uav.guided_mode()
        if action_succeed == self.uav.ACTION_SUCCESS:
            if self._apply_operator_effect(action_dispatch.name,
                                           action_dispatch.parameters):
                self.publish_feedback(action_dispatch.action_id,
                                      'action achieved')
            else:
                self.publish_feedback(action_dispatch.action_id,
                                      'action failed')
        else:
            self.publish_feedback(action_dispatch.action_id, 'action failed')

    def takeoff(self, action_dispatch):
        """
        UAV takeoff action interfacing with ROSPlan
        """
        self.publish_feedback(action_dispatch.action_id, 'action enabled')
        start_time = rospy.Time(action_dispatch.dispatch_time)
        duration = rospy.Duration(action_dispatch.duration)
        self._rate.sleep()
        rospy.loginfo('Dispatching setting mode to %s action ...' %
                      action_dispatch.name)
        rospy.loginfo('Dispatch action at %s with duration %s' %
                      (str(start_time.secs), str(duration.to_sec())))

        action_succeed = self.uav.takeoff(
            rospy.get_param('~takeoff_altitude', 10.))
        if action_succeed == self.uav.ACTION_SUCCESS:
            if self._apply_operator_effect(action_dispatch.name,
                                           action_dispatch.parameters):
                self.publish_feedback(action_dispatch.action_id,
                                      'action achieved')
            else:
                self.publish_feedback(action_dispatch.action_id,
                                      'action failed')
        else:
            self.publish_feedback(action_dispatch.action_id, 'action failed')

    def rtl(self, action_dispatch):
        """
        UAV return to launch action interfacing with ROSPlan
        """
        self.publish_feedback(action_dispatch.action_id, 'action enabled')
        start_time = rospy.Time(action_dispatch.dispatch_time)
        duration = rospy.Duration(action_dispatch.duration)
        self._rate.sleep()
        rospy.loginfo('Dispatching setting mode to %s action ...' %
                      action_dispatch.name)
        rospy.loginfo('Dispatch action at %s with duration %s' %
                      (str(start_time.secs), str(duration.to_sec())))
        action_succeed = self.uav.return_to_land(monitor_home=False)
        if action_succeed == self.uav.ACTION_SUCCESS:
            if self._apply_operator_effect(action_dispatch.name,
                                           action_dispatch.parameters):
                self.publish_feedback(action_dispatch.action_id,
                                      'action achieved')
            else:
                self.publish_feedback(action_dispatch.action_id,
                                      'action failed')
        else:
            self.publish_feedback(action_dispatch.action_id, 'action failed')

    def goto_waypoint(self, action_dispatch):
        """
        UAV return to launch action interfacing with ROSPlan
        """
        self.publish_feedback(action_dispatch.action_id, 'action enabled')
        start_time = rospy.Time(action_dispatch.dispatch_time)
        duration = rospy.Duration(action_dispatch.duration)
        self._rate.sleep()
        rospy.loginfo('Dispatching setting mode to %s action ...' %
                      action_dispatch.name)
        rospy.loginfo('Dispatch action at %s with duration %s' %
                      (str(start_time.secs), str(duration.to_sec())))
        waypoint = -1
        for param in action_dispatch.parameters:
            if param.key == 'to':
                waypoint = int(param.value)
                break
        action_succeed = self.uav.goto(self, waypoint)
        if action_succeed == self.uav.ACTION_SUCCESS:
            if self._apply_operator_effect(action_dispatch.name,
                                           action_dispatch.parameters):
                self.publish_feedback(action_dispatch.action_id,
                                      'action achieved')
            else:
                self.publish_feedback(action_dispatch.action_id,
                                      'action failed')
        else:
            self.publish_feedback(action_dispatch.action_id, 'action failed')
