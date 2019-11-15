#!/usr/bin/env python

# 3rd Party Packages
import sys
import argparse

import roslib
# ROS Packages
import rospy
from mavros_msgs.msg import HomePosition
from mavros_msgs.srv import CommandHome
from pygeodesy import GeoidKarney
from uav_rosplan.interface import UAVActionInterface
from uav_rosplan.preflightcheck import PreFlightCheck

seq = 1
home_poses = list()


def home_cb(msg):
    global home
    home = msg


def create_home_poses():
    global home
    global home_poses
    geoid_interpolator = GeoidKarney(
        roslib.packages.get_pkg_dir('uav_rosplan') + '/config/egm96-5.pgm')
    for i in range(-1, 2):
        new_lat = home.geo.latitude + (i * 1e-05)
        new_long = home.geo.longitude + (i * 1e-05)
        new_amsl = home.geo.altitude - geoid_interpolator.height(
            home.geo.latitude, home.geo.longitude)
        home_poses.append((new_lat, new_long, new_amsl))


def move_home(event):
    global seq
    global home_poses
    global set_home_proxy
    seq += 1
    if not (seq % len(home_poses)):
        home_poses.reverse()
        seq += 1
    new_lat = home_poses[seq % len(home_poses)][0]
    new_long = home_poses[seq % len(home_poses)][1]
    new_alt = home_poses[seq % len(home_poses)][2]
    resp = set_home_proxy(False, new_lat, new_long, new_alt)
    if resp.success:
        rospy.loginfo('New home pose: (%.7f, %.7f)' % (new_lat, new_long))


if __name__ == '__main__':
    global set_home_proxy
    rospy.init_node('uav_mission_test')
    parser_arg = argparse.ArgumentParser(prog=rospy.get_name())
    parser_arg.add_argument('mavlink_conn', help='Mavlink Connection')
    parser_arg.add_argument("-f",
                            dest='wp_file',
                            default='wp_config.yaml',
                            help='waypoint\'s file name')
    parser_arg.add_argument('-m',
                            dest='home',
                            default='0',
                            help='Monitoring home (Yes[1] / No[0])')
    parser_arg.add_argument('-wp',
                            dest='waypoint',
                            default='0',
                            help='Go to specific waypoint')
    args = parser_arg.parse_args()
    rospy.set_param('~wp_file', args.wp_file)
    # Initialise UAV
    uav = UAVActionInterface()
    preflightchecked = False
    for _ in range(3):
        try:
            preflightcheck = PreFlightCheck(args.mavlink_conn, uav)
            preflightchecked = True
        except RuntimeError:
            preflightchecked = False
        if preflightchecked:
            break
    try:
        init_batt = float(preflightcheck._init_batt.get())
        min_batt = float(preflightcheck._low_batt.get())
        uav.set_battery(init_batt, min_batt)
    except ValueError:
        rospy.logwarn('Initial / Minimum Battery can\'t be set!')
        sys.exit(2)

    rospy.loginfo('Waiting for the UAV to be ARMED ...')
    while not uav.state.armed:
        uav._rate.sleep()
    rospy.loginfo('Enabling guided mode: %d' % uav.guided_mode())
    rospy.loginfo('Taking off: %d' %
                  uav.takeoff(rospy.get_param('~takeoff_altitude', 10.)))
    if not uav.low_battery:
        for _ in range(20):
            uav._rate.sleep()

        if bool(int(args.home)):
            rospy.Subscriber('/mavros/home_position/home',
                             HomePosition,
                             home_cb,
                             queue_size=10)
            rospy.loginfo('Waiting for /mavros/cmd/set_home service ...')
            rospy.wait_for_service('/mavros/cmd/set_home')
            set_home_proxy = rospy.ServiceProxy('/mavros/cmd/set_home',
                                                CommandHome)
            create_home_poses()
            rospy.Timer(rospy.Duration(7), move_home)
        if int(args.waypoint) == 0:
            rospy.loginfo('Full mission completed: %d' %
                          uav.full_mission_auto(1, rospy.Duration(600, 0)))
        else:
            rospy.loginfo('Go to completed: %d' % uav.goto(int(args.waypoint)))
    rospy.loginfo('RTL success: %d' %
                  uav.return_to_land(monitor_home=bool(int(args.home))))
    rospy.loginfo('Waiting for motor disarming ...')
    while uav.state.armed:
        uav._rate.sleep()
    rospy.loginfo('Motor disarm ...')
