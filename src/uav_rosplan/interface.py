#!/usr/bin/env python

import numpy as np
import roslib
import rospy
import yaml
from mavros_msgs.msg import (HomePosition, State, Waypoint, WaypointReached)
from mavros_msgs.srv import (CommandBool, CommandHome, CommandLong, CommandTOL,
                             SetMode, WaypointClear, WaypointPush,
                             WaypointSetCurrent)
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Float64, Header


class UAVActionInterface(object):

    INIT_VOLTAGE = 12.587
    MINIMUM_VOLTAGE = 12.19
    EXTERNAL_INTERVENTION = -2
    OUT_OF_DURATION = -1
    ACTION_SUCCESS = 1
    ACTION_FAIL = 0

    def __init__(self, update_frequency=10.):
        """
        A Class that interfaces ROSPlan and MAVROS for executing actions
        """
        self.landed = True
        self.home_moved = False
        self.rel_alt = 0.
        self.wp_reached = -1
        self.previous_mode = ''
        self.current_mode = ''
        self.gcs_intervention = False
        self.state = State()
        self.waypoints = list()
        self.home = HomePosition()
        self.compass_hdg = Float64()
        self.global_pose = NavSatFix()
        self.battery_voltages = [self.INIT_VOLTAGE for _ in range(30)]
        self.low_battery = False
        self._rel_alt = [0. for _ in range(15)]
        self._rel_alt_seq = 0

        # Service proxies
        rospy.loginfo('Waiting for service /mavros/cmd/command ...')
        rospy.wait_for_service('/mavros/cmd/command')
        self._command_proxy = rospy.ServiceProxy('/mavros/cmd/command',
                                                 CommandLong)
        rospy.loginfo('Waiting for service /mavros/mission/push ...')
        rospy.wait_for_service('/mavros/mission/push')
        self._add_wp_proxy = rospy.ServiceProxy('/mavros/mission/push',
                                                WaypointPush)
        rospy.loginfo('Waiting for service /mavros/mission/set_current ...')
        rospy.wait_for_service('/mavros/mission/set_current')
        self._set_current_wp_proxy = rospy.ServiceProxy(
            '/mavros/mission/set_current', WaypointSetCurrent)
        rospy.loginfo('Waiting for /mavros/set_mode ...')
        rospy.wait_for_service('/mavros/set_mode')
        self._set_mode_proxy = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        rospy.loginfo('Wait for service /mavros/mission/clear ...')
        rospy.wait_for_service('/mavros/mission/clear')
        self._clear_wp_proxy = rospy.ServiceProxy('/mavros/mission/clear',
                                                  WaypointClear)
        rospy.loginfo('Waiting for /mavros/cmd/arming ...')
        rospy.wait_for_service('/mavros/cmd/arming')
        self._arming_proxy = rospy.ServiceProxy('/mavros/cmd/arming',
                                                CommandBool)
        rospy.loginfo('Waiting for /mavros/cmd/takeoff ...')
        rospy.wait_for_service('/mavros/cmd/takeoff')
        self._takeoff_proxy = rospy.ServiceProxy('/mavros/cmd/takeoff',
                                                 CommandTOL)
        rospy.loginfo('Waiting for /mavros/cmd/set_home ...')
        rospy.wait_for_service('/mavros/cmd/set_home')
        self._set_home_proxy = rospy.ServiceProxy('/mavros/cmd/set_home',
                                                  CommandHome)

        self._rate = rospy.Rate(update_frequency)

        # Subscribers
        rospy.Subscriber('/mavros/state', State, self._state_cb, queue_size=10)
        # halt until mavros is connected to a uav
        rospy.loginfo('Waiting for a connection between MAVROS and UAV ...')
        while (not self.state.connected):
            self._rate.sleep()
        rospy.Subscriber('/mavros/home_position/home',
                         HomePosition,
                         self._home_cb,
                         queue_size=10)
        rospy.Subscriber('/mavros/global_position/rel_alt',
                         Float64,
                         self._relative_alt_cb,
                         queue_size=10)
        rospy.Subscriber('/mavros/battery',
                         BatteryState,
                         self._battery_cb,
                         queue_size=10)
        rospy.Subscriber('/mavros/global_position/global',
                         NavSatFix,
                         self._global_pose_cb,
                         queue_size=10)
        rospy.Subscriber('/mavros/global_position/compass_hdg',
                         Float64,
                         self._compass_hdg_cb,
                         queue_size=10)
        rospy.Subscriber('/mavros/mission/reached',
                         WaypointReached,
                         self._wp_reached_cb,
                         queue_size=10)

        # Auto call functions
        rospy.Timer(rospy.Duration(1. / update_frequency),
                    self.update_landing_status)
        rospy.Timer(rospy.Duration(20. / update_frequency),
                    self.overwatch_current_mode)
        rospy.loginfo('Adding WPs ...')
        self._wait(10)
        # Adding initial waypoints' configuration
        while not self.add_waypoints():
            self._rate.sleep()

    def _wait(self, n_rate):
        """
        Sleep for n times rate
        """
        for _ in range(n_rate):
            self._rate.sleep()

    def _wp_reached_cb(self, msg):
        """
        Waypoint reached call back
        """
        self.wp_reached = msg.wp_seq

    def _compass_hdg_cb(self, msg):
        """
        Compass heading from global position Mavros callback
        """
        self.compass_hdg = msg.data

    def _state_cb(self, msg):
        """
        Mavros state callback
        """
        if self.current_mode == '':
            self.current_mode = msg.mode
        self.state = msg

    def _home_cb(self, msg):
        """
        Home position callback
        """
        if self.home.header != Header():
            lat_change = abs(self.home.geo.latitude - msg.geo.latitude) > 5e-06
            long_change = abs(self.home.geo.longitude -
                              msg.geo.longitude) > 5e-06
            self.home_moved = lat_change or long_change
            if self.home_moved:
                rospy.logwarn(
                    'Home has moved from (%.6f, %.6f) to (%.6f, %.6f)' %
                    (self.home.geo.latitude, self.home.geo.longitude,
                     msg.geo.latitude, msg.geo.longitude))
        self.home = msg

    def _relative_alt_cb(self, msg):
        """
        Relative altitude callback
        """
        self._rel_alt[self._rel_alt_seq % 15] = msg.data
        self._rel_alt_seq += 1
        self.rel_alt = np.mean(self._rel_alt)

    def _global_pose_cb(self, msg):
        """
        UAV global position callback
        """
        self.global_pose = msg

    def _battery_cb(self, msg):
        """
        UAV Battery state callback
        """
        self.battery_voltages[msg.header.seq % 30] = msg.voltage
        self.low_battery = (
            sum(self.battery_voltages) / float(len(self.battery_voltages)) <=
            self.MINIMUM_VOLTAGE)

    def set_battery(self, init_batt, min_batt):
        """
        Setting initial battery and minimum battery condition
        """
        rospy.loginfo('Setting up battery requirements...')
        self.INIT_VOLTAGE = init_batt
        self.MINIMUM_VOLTAGE = min_batt
        self.battery_voltages = [self.INIT_VOLTAGE for _ in range(30)]
        self.low_battery = False

    def overwatch_current_mode(self, event):
        """
        Watch whether human operator intervenes
        """
        mode_status_check = (self.current_mode != '') and (
            self.state.mode not in [self.current_mode, self.previous_mode])
        stabilize_on_land_check = (
            self.state.mode == 'STABILIZE') and self.landed
        self.gcs_intervention = mode_status_check and (
            not stabilize_on_land_check)

    def update_landing_status(self, event):
        """
        Automated update landing (or flying) status
        """
        self.landed = (not self.state.armed) or (self.rel_alt <= 0.1)

    def load_wp_config_from_file(self):
        """
        Load waypoints configuration from file
        located at uav_rosplan package under config folder
        """
        pkg_path = roslib.packages.get_pkg_dir('uav_rosplan')
        wp_file = rospy.get_param('~wp_file', 'wp_config.yaml')
        waypoints = yaml.load(open(pkg_path + '/config/%s' % wp_file, 'r'))
        return waypoints

    def add_waypoints(self, index=0):
        """
        Setting up waypoints for the UAV
        """
        if self.home.header == Header():
            rospy.logwarn(
                'Home has not been set! Setting current location as home.')
            self.set_current_location_as_home()
        # Clear current wps available
        self._clear_wp_proxy()
        # First wp is home with landing action
        home_wp = Waypoint(Waypoint.FRAME_GLOBAL_REL_ALT, 21, False, False, .6,
                           1., 0, np.float('nan'), self.home.geo.latitude,
                           self.home.geo.longitude, 0.)
        wps = [home_wp]
        # Load waypoints and start adding them to the list
        waypoints = self.load_wp_config_from_file()
        for waypoint in waypoints['waypoints']:
            wp = Waypoint(Waypoint.FRAME_GLOBAL_REL_ALT, 16, False, False,
                          waypoint['hold_time'], waypoint['radius'], 0.,
                          waypoint['yaw'], waypoint['lat'], waypoint['long'],
                          waypoint['rel_alt'])
            wps.append(wp)
        # Push waypoints to mavros service
        response = self._add_wp_proxy(index, wps)
        if response.success:
            rospy.loginfo('%d waypoints are added' % response.wp_transfered)
        else:
            rospy.logwarn('No waypoint is added!')
        self.waypoints = wps
        return response.success

    def set_current_target_wp(self, wp_index):
        """
        Set target wp for UAV to go
        """
        response = self._set_current_wp_proxy(wp_index)
        if response.success:
            rospy.loginfo('Setting current target waypoint to %d' % wp_index)
        else:
            rospy.logwarn('Waypoint %d can\' be set!' % wp_index)
        return response.success

    def set_current_location_as_home(self):
        """
        Set current location as new home location
        """
        response = self._set_home_proxy(True, 0., 0., 0.)
        if response.success:
            rospy.loginfo('Setting current location as new home ...')
        else:
            rospy.logwarn('Current location can\'t be set as the new home!')
        return response.success

    def guided_mode(self, duration=rospy.Duration(60, 0)):
        """
        Guided mode action
        """
        start = rospy.Time.now()
        guided = False
        if self.state.mode != 'GUIDED':
            while (rospy.Time.now() - start <
                   duration) and (not rospy.is_shutdown()) and (
                       not guided) and (not self.gcs_intervention):
                guided = self._set_mode_proxy(0, 'guided').mode_sent
                if guided:
                    self.previous_mode = self.current_mode
                    self.current_mode = 'GUIDED'
                self._rate.sleep()
            rospy.loginfo('Changing mode to GUIDED ...')
        else:
            self.previous_mode = self.current_mode
            self.current_mode = 'GUIDED'
            guided = True
        if (rospy.Time.now() - start) > duration:
            guided = self.OUT_OF_DURATION
        if self.gcs_intervention:
            guided = self.EXTERNAL_INTERVENTION
        return int(guided)

    def arm(self, duration=rospy.Duration(60, 0)):
        """
        Arm throttle action
        """
        start = rospy.Time.now()
        self.guided_mode(duration)
        armed = False
        if not self.state.armed:
            while (rospy.Time.now() - start <
                   duration) and (not rospy.is_shutdown()) and (
                       not armed) and (not self.gcs_intervention):
                armed = self._arming_proxy(True).success
                self._rate.sleep()
            rospy.loginfo('Arming the UAV ...')
        else:
            armed = True
        response = int(armed)
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        if self.gcs_intervention:
            response = self.EXTERNAL_INTERVENTION
        return response

    def takeoff(self, altitude, duration=rospy.Duration(60, 0)):
        """
        Take off action
        """
        start = rospy.Time.now()
        self.guided_mode(duration)
        took_off = True
        if self.landed:
            took_off = self._takeoff_proxy(0.1, 0, 0, 0, altitude).success
            rospy.loginfo('UAV is taking off to %d meter height' % altitude)
            while (rospy.Time.now() - start <
                   duration) and (not rospy.is_shutdown()
                                  ) and (self.rel_alt - altitude) < -0.5 and (
                                      not self.gcs_intervention):
                if self.low_battery:
                    rospy.logwarn('Battery is below minimum voltage!')
                    break
                if not took_off:
                    took_off = self._takeoff_proxy(0.1, 0, 0, 0,
                                                   altitude).success
                self._rate.sleep()
        response = int(took_off)
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        if self.gcs_intervention:
            response = self.EXTERNAL_INTERVENTION
        return response

    def goto(self, waypoint, duration=rospy.Duration(60, 0)):
        """
        Go to specific waypoint action
        """
        self.wp_reached = -1
        start = rospy.Time.now()
        if not self.set_current_target_wp(waypoint):
            return self.ACTION_FAIL
        auto = False
        while (rospy.Time.now() - start <
               duration) and (not rospy.is_shutdown()) and (
                   not self.gcs_intervention) and (not auto):
            auto = self._set_mode_proxy(0, 'auto').mode_sent
            if auto:
                self.previous_mode = self.current_mode
                self.current_mode = 'AUTO'
            self._rate.sleep()
        rospy.loginfo('Setting mode to AUTO ...')
        while (rospy.Time.now() - start < duration) and not (
                rospy.is_shutdown()) and (not self.gcs_intervention) and (
                    (waypoint != self.wp_reached)):
            if self.low_battery:
                rospy.logwarn('Battery is below minimum voltage!')
                break
            self._rate.sleep()
        response = int(waypoint == self.wp_reached)
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        elif self.gcs_intervention:
            response = self.EXTERNAL_INTERVENTION
        return response

    def full_mission_auto(self, waypoint, duration=rospy.Duration(60, 0)):
        """
        Go from waypoint i to the last
        """
        self.wp_reached = -1
        wps_reached = list()
        start = rospy.Time.now()
        if not self.set_current_target_wp(waypoint):
            return self.ACTION_FAIL
        auto = False
        while (rospy.Time.now() - start <
               duration) and (not rospy.is_shutdown()) and (
                   not self.gcs_intervention) and len(wps_reached) < len(
                       self.waypoints[waypoint:]):
            if not auto:
                rospy.loginfo('Setting mode to AUTO ...')
                auto = self._set_mode_proxy(0, 'auto').mode_sent
                self.previous_mode = self.current_mode if auto else (
                    self.previous_mode)
                self.current_mode = 'AUTO' if auto else self.current_mode
            if self.wp_reached != -1 and self.wp_reached not in wps_reached:
                wps_reached.append(self.wp_reached)
            if self.low_battery:
                rospy.logwarn('Battery is below minimum voltage!')
                break
            self._rate.sleep()
        response = int(len(wps_reached) == len(self.waypoints[waypoint:]))
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        elif self.gcs_intervention:
            response = self.EXTERNAL_INTERVENTION
        return response

    def return_to_land(self,
                       monitor_home=False,
                       duration=rospy.Duration(600, 0)):
        """
        Return to home action
        """
        rtl_set = False
        emergency_landing = False
        start = rospy.Time.now()
        for i in range(3):
            if self.gcs_intervention:
                break
            rtl_set = self._set_mode_proxy(0, 'rtl').mode_sent
            if rtl_set:
                rospy.loginfo('Setting mode to RTL ...')
                self.previous_mode = self.current_mode
                self.current_mode = 'RTL'
                break
            self._rate.sleep()
        # rtl_set = False
        while (rospy.Time.now() - start <
               duration) and not (rospy.is_shutdown()) and (
                   not self.gcs_intervention) and (not self.landed):
            cond = monitor_home and self.home_moved
            if cond or (not rtl_set and not emergency_landing):
                emergency_landing = self.emergency_landing()
        rtl_set = (self.landed and
                   (rtl_set or (emergency_landing and self.wp_reached == 1)))
        if (rospy.Time.now() - start) > duration:
            rtl_set = self.OUT_OF_DURATION
        if self.gcs_intervention:
            rtl_set = self.EXTERNAL_INTERVENTION
        if emergency_landing:
            self.add_waypoints()
        return int(rtl_set)

    def emergency_landing(self):
        """
        Emergency land to home when home wobbles
        """
        # Clear current wps available
        self._clear_wp_proxy()
        # wps[0] must be home waypoint
        home_wp = Waypoint(Waypoint.FRAME_GLOBAL_REL_ALT, 21, False, False,
                           1.0, 1., 0, np.float('nan'), self.home.geo.latitude,
                           self.home.geo.longitude, 0.)
        wps = [home_wp, home_wp]
        self.waypoints = wps
        # Push waypoints to mavros service
        if self._add_wp_proxy(0, wps).success:
            self.home_moved = False
            # assume that set_current_wp_proxy sets wp to 1
            self.full_mission_auto(1, rospy.Duration(1, 0))
        return True

    def reboot_autopilot_computer(self, duration=rospy.Duration(60, 0)):
        """
        Preflight reboot shutdown for autopilot and onboard computer
        """
        start = rospy.Time.now()
        result = self._command_proxy(broadcast=False,
                                     command=246,
                                     confirmation=0,
                                     param1=1.,
                                     param2=1.,
                                     param3=0.,
                                     param4=0.,
                                     param5=0.,
                                     param6=0.,
                                     param7=0.)
        response = self.ACTION_SUCCESS if result.success else self.ACTION_FAIL
        if (rospy.Time.now() - start) > duration:
            response = self.OUT_OF_DURATION
        return response
