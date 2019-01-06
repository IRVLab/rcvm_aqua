#! /usr/bin/python

import sys, math, threading
from time import sleep
from math import pi

import rospy

from aquacore.msg import AutopilotModes
from rcvm_pilot_client import RCVMPilotClient

from rcvm_core.srv import Affirmative, Attention, Danger, FollowMe, IndicateMovement, IndicateObject
from rcvm_core.srv import IndicateStay, Lost, Malfunction, Negative, Possibly, RepeatLast, ReportBattery

from timeout import Timeout

rospy.init_node('rcvm_server', argv=None, anonymous=True)
params = {}
#params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_FIXED_DEPTH
params['mode'] = AutopilotModes.AP_GLOBAL_ANGLES_LOCAL_THRUST
pc = RCVMPilotClient(params)

'''
    Service handlers.

    For PC relative angle change, order is [RPY] in degrees.
'''
def affirmative_handler(req):
    d = pc.current_depth
    vx = 0 
    vz = 0

    # Nod robot up and down (pitches of 20 deg from center)
    try:
        pc.do_relative_angle_change([0, 20, 0], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([0, -40, 0], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([0, 40, 0], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([0, -20, 0], d, vx, vz, time_sec=10)
    except Timeout.Timeout:
        return False

    return True

def attention_handler(req):
    d = pc.current_depth
    vx = 0 
    vz = 0

    # Roll robot by 60 deg from center, with small pitch and yaw (5 deg from center)
    try:
        pc.do_relative_angle_change([60, 5, 5], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([-120, -10, -10], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([120, 10, 10], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([-60, -5, -5], d, vx, vz, time_sec=10)
    except Timeout.Timeout:
        return False

    return True
    

def danger_handler(req):
    d = pc.current_depth
    vx = 0 
    vz = 0

    #Swim robot forward for a moment, look back and forth, swim forward again, vehemently "shake head"
    try:
        rads = pc.get_rpy_of_imu_in_global()
        degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
        pc.do_straight_line(5, degs, d, 0.3, vz)

        pc.do_relative_angle_change([0, 0, 90], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([0, 0, -180], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([0, 0, 90], d, vx, vz, time_sec=10)

        rads = pc.get_rpy_of_imu_in_global()
        degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
        pc.do_straight_line(5, degs, d, 0.3, vz)

        pc.do_relative_angle_change([0, 0, 15], d, 0.3, vz, time_sec=10)
        pc.do_relative_angle_change([0, 0, -30], d, 0.3, vz, time_sec=10)
        pc.do_relative_angle_change([0, 0, 30], d, 0.3, vz, time_sec=10)
        pc.do_relative_angle_change([0, 0, -30], d, 0.3, vz, time_sec=10)
        pc.do_relative_angle_change([0, 0, 15], d, 0.3, vz, time_sec=10)
    except Timeout.Timeout:
        return False

    return True

def follow_me_handler(req):
    d = pc.current_depth
    vx = 0 
    vz = 0

    # Make a beckoning gesture with head, swim forward.
    try:
        pc.do_relative_angle_change([20, 5, 30], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([-20, -5, -30], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([20, 5, 30], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([-20, -5, 150], d, vx, vz, time_sec=10)

        rads = pc.get_rpy_of_imu_in_global()
        degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
        pc.do_straight_line(5, degs, d, vx, vz)
    except Timeout.Timeout:
        return False

    return True

def indicate_movement_handler(req):
    pass

def indicate_object_handler(req):
    pass

def indicate_stay_handler(req):
    d = pc.current_depth
    vx = 0.25
    vz = 0

    # Make a squarish circle by doing 90 degree turns while moving forward, with straight motion in between.
    # TODO: Smooth this out?
    try:
        pc.do_relative_angle_change([0,0,90], d, vx, vz, time_sec=5)
        rads = pc.get_rpy_of_imu_in_global()
        degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
        pc.do_straight_line(5, degs, d, vx, vz)

        pc.do_relative_angle_change([0,0,90], d, vx, vz, time_sec=5)
        rads = pc.get_rpy_of_imu_in_global()
        degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
        pc.do_straight_line(5, degs, d, vx, vz)

        pc.do_relative_angle_change([0,0,90], d, vx, vz, time_sec=5)
        rads = pc.get_rpy_of_imu_in_global()
        degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
        pc.do_straight_line(5, degs, d, vx, vz)

        pc.do_relative_angle_change([0,0,90], d, vx, vz, time_sec=5)
        rads = pc.get_rpy_of_imu_in_global()
        degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
        pc.do_straight_line(5, degs, d, vx, vz)

    except Timeout.Timeout:
        return False

    return True

def lost_handler(req):
    d = pc.current_depth
    vx = 0.2 
    vz = 0

    # Wander the robot forward, looking back and forth. Yaws of 70 degrees
    try:
        pc.do_relative_angle_change([0, 0, 70], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([0, 0, -140], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([0, 0,  140], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([0, 0, -140], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([0, 0, 70], d, vx, vz, time_sec=10)
    except Timeout.Timeout:
        return False
    
    return True

def malfunction_handler(req):
    d = pc.current_depth
    vx = 0.2 
    vz = 0.1

    # Belly-up the robot (roll 180 deg), then move forwards and up slowly.
    try:
        pc.do_relative_angle_change([180, 0, 0], d, vx, vz, time_sec=10)
        rads = pc.get_rpy_of_imu_in_global()
        degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
        pc.do_straight_line(5, degs, d-1, vx, 0.3)
    except Timeout.Timeout:
        return False
    
    return True
    

def negative_handler(req):
    d = pc.current_depth
    vx = 0 
    vz = 0

    # Shake the robot's "head".  Yaws of 15 degrees from center.
    try:
        pc.do_relative_angle_change([0, 0, 15], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([0, 0, -30], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([0, 0, 30], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([0, 0, -15], d, vx, vz, time_sec=10)
    except Timeout.Timeout:
        return False
    
    return True
            
def possibly_handler(req):
    d = pc.current_depth
    vx = 0 
    vz = 0

    # Bobble the robot's "head", Rolls of 15 degrees off center.
    try:
        pc.do_relative_angle_change([ 15, 0, 0], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([-30, 0, 0], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([ 30, 0, 0], d, vx, vz, time_sec=10)
        pc.do_relative_angle_change([-15, 0, 0], d, vx, vz, time_sec=10)
    except Timeout.Timeout:
        return False
    
    return True

def repeat_last_handler(req):
    d = pc.current_depth
    vx = 0 
    vz = 0

    # "Cock an ear" by doing a 20, 10, 15 RPY, then go back after 2 seconds.
    try:
        pc.do_relative_angle_change([20, 10, 15], d, vx, vz, time_sec=10)
        sleep(2)
        pc.do_relative_angle_change([-20, -10, -15], d, vx, vz, time_sec=10)
    except Timeout.Timeout:
        return False

    return True

def report_battery_handler(req):
    pass


if __name__ == "__main__":
    rospy.loginfo('Initializing Aqua8 RCVM server...')

    rospy.Service('/rcvm/affirmative', Affirmative, affirmative_handler)
    rospy.Service('/rcvm/attention', Attention, attention_handler)
    rospy.Service('/rcvm/danger', Danger, danger_handler)
    rospy.Service('/rcvm/follow_me', FollowMe, follow_me_handler)
    rospy.Service('/rcvm/indicate_movement', IndicateMovement, indicate_movement_handler)
    rospy.Service('/rcvm/indicate_object', IndicateObject, indicate_object_handler)
    rospy.Service('/rcvm/indicate_stay', IndicateStay, indicate_stay_handler)
    rospy.Service('/rcvm/lost', Lost, lost_handler)
    rospy.Service('/rcvm/malfunction', Malfunction, malfunction_handler)
    rospy.Service('/rcvm/negative', Negative, negative_handler)
    rospy.Service('/rcvm/possibly', Possibly, possibly_handler)
    rospy.Service('/rcvm/repeat_last', RepeatLast, repeat_last_handler)
    rospy.Service('/rcvm/report_battery', ReportBattery, report_battery_handler)

    rospy.loginfo('      Service advertising completed...')
    rospy.loginfo('RCVM server ready for business!')
    rospy.loginfo('Spinning forever until a service request is recieved.')    

    # Spin forever to avoid early shutdown.
    rate = rospy.Rate(60)
    while not rospy.is_shutdown():
        rate.sleep()
        
else:
    pass
