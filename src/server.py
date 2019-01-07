#! /usr/bin/python

import sys, math, threading, signal
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

    Roll: Positive is clockwise, negative is counter clockwise.
    Yaw: Positive is left, negative is right.
    Pitch: Positive is down, negative is up.

    Speed: Positive is the only option.
    Heave: Positive is up, negative is down.


    TODO: Create timeouts handler to deal with autopilot failures.
'''
def affirmative_handler(req):
    d = pc.current_depth
    vx = 0 
    vz = 0

    # Nod robot up and down (pitches of 20 deg from center)
    rospy.loginfo(' [AFFIRMATIVE]: Initiating kinme.')
    rospy.loginfo(' [AFFIRMATIVE]: Pitch down...')
    pc.do_relative_angle_change([0, 20, 0], d, vx, vz)
    rospy.loginfo(' [AFFIRMATIVE]: Now pitch up...')
    pc.do_relative_angle_change([0, -40, 0], d, vx, vz)
    rospy.loginfo(' [AFFIRMATIVE]: Now pitch down again...')
    pc.do_relative_angle_change([0, 40, 0], d, vx, vz)
    rospy.loginfo(' [AFFIRMATIVE]: Now pitch up...')
    pc.do_relative_angle_change([0, -20, 0], d, vx, vz)
    rospy.loginfo(' [AFFIRMATIVE]: Kineme complete!')

    return True

def attention_handler(req):
    d = pc.current_depth
    vx = 0 
    vz = 0

    # Roll robot by 60 deg from center, with small pitch and yaw (5 deg from center)

    rospy.loginfo('  [ATTENTION]: Initiating kineme.')
    rospy.loginfo('  [ATTENTION]: Roll clockwise..')
    pc.do_relative_angle_change([60, 5, 5], d, vx, vz)
    rospy.loginfo('  [ATTENTION]: Roll counter clockwise.')
    pc.do_relative_angle_change([-120, -10, -10], d, vx, vz)
    rospy.loginfo('  [ATTENTION]: Roll clockwise.')
    pc.do_relative_angle_change([120, 10, 10], d, vx, vz)
    rospy.loginfo('  [ATTENTION]: Roll counter clockwise.')
    pc.do_relative_angle_change([-60, -5, -5], d, vx, vz)
    rospy.loginfo('  [ATTENTION]: Kineme complete!')

    return True
    

def danger_handler(req):
    d = pc.current_depth
    vx = 0 
    vz = 0

    #Swim robot forward for a moment, look back and forth, swim forward again, vehemently "shake head"
    rospy.loginfo('  [DANGER]: Initiated kineme.')
    rospy.loginfo('  [DANGER]: Forward for 2 seconds.')
    rads = pc.get_rpy_of_imu_in_global()
    degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
    pc.do_straight_line(2, degs, d, 0.3, vz)

    rospy.loginfo('  [DANGER]: Look left.')
    pc.do_relative_angle_change([0, 0, 90], d, vx, vz)
    rospy.loginfo('  [DANGER]: Look right.')
    pc.do_relative_angle_change([0, 0, -90], d, vx, vz)
    pc.do_relative_angle_change([0, 0, -90], d, vx, vz)
    rospy.loginfo('  [DANGER]: Look back left to center.')
    pc.do_relative_angle_change([0, 0, 90], d, vx, vz)

    rospy.loginfo('  [DANGER]: Forward for another 2 seconds.')
    rads = pc.get_rpy_of_imu_in_global()
    degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
    pc.do_straight_line(2, degs, d, 0.3, vz)

    rospy.loginfo('  [DANGER]: Shake head.')
    pc.do_relative_angle_change([0, 0, 15], d, 0.3, vz)
    pc.do_relative_angle_change([0, 0, -30], d, 0.3, vz)
    pc.do_relative_angle_change([0, 0, 30], d, 0.3, vz)
    pc.do_relative_angle_change([0, 0, -30], d, 0.3, vz)
    pc.do_relative_angle_change([0, 0, 15], d, 0.3, vz)
    rospy.loginfo('  [DANGER]: Kineme completed!')

    return True

def follow_me_handler(req):
    d = pc.current_depth
    vx = 0 
    vz = 0

    # Make a beckoning gesture with head, swim forward.
    rospy.loginfo('  [FOLLOW_ME]: Kineme initiated.')
    rospy.loginfo('  [FOLLOW_ME]: Beckon back')
    pc.do_relative_angle_change([-20, -5, 30], d, vx, vz)
    rospy.loginfo('  [FOLLOW_ME]: And back to center.')
    pc.do_relative_angle_change([20, 5, -30], d, vx, vz)
    rospy.loginfo('  [FOLLOW_ME]: Back once more')
    pc.do_relative_angle_change([-20, -5, 30], d, vx, vz)
    rospy.loginfo('  [FOLLOW_ME]: Back to center and turn around.')
    pc.do_relative_angle_change([20, 5, 150], d, vx, vz)

    rospy.loginfo('  [FOLLOW_ME]: Forward (away from diver) for 2 seconds.')
    rads = pc.get_rpy_of_imu_in_global()
    degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
    pc.do_straight_line(2, degs, d, vx, vz)

    rospy.loginfo('  [FOLLOW_ME]: Kineme completed!')
    
    return True

# TODO: Respond to movement vector. Right now it just indicates down.
def indicate_movement_handler(req):
    d = pc.current_depth
    vx = 0 
    vz = 0

    rospy.loginfo('  [INDICATE_MOVEMENT]: Kineme initiated.')
    rospy.loginfo('  [INDICATE_MOVEMENT]: Pitch down and move forward.')
    pc.do_relative_angle_change([0,30,0], d, 0.25, vz)
    rospy.loginfo('  [INDICATE_MOVEMENT]: Pitch back up to look at diver.')
    pc.do_relative_angle_change([0,-60,0], d, vx, vz)
    rospy.loginfo('  [INDICATE_MOVEMENT]: Pitch back down.')
    pc.do_relative_angle_change([0,60,0], d, vx, vz)

    rospy.loginfo('  [INDICATE_MOVEMENT]: Move foreward for 2 seconds.')
    rads = pc.get_rpy_of_imu_in_global()
    degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
    pc.do_straight_line(2, degs, d, 0.3, vz)

    rospy.loginfo('  [INDICATE_MOVEMENT]: Kineme completed!')
    
    return True

# TODO: Respond to object orientation.
def indicate_object_handler(req):
    d = pc.current_depth
    vx = 0 
    vz = 0

    rospy.loginfo('  [INDICATE_OBJECT]: Kineme initiated.')
    rospy.loginfo('  [INDICATE_OBJECT]: Look towards object and move forward.')
    pc.do_relative_angle_change([0,15,15], d, 0.25, vz)
    rospy.loginfo('  [INDICATE_OBJECT]: Look back at diver.')
    pc.do_relative_angle_change([0,-30,-30], d, vx, vz)
    rospy.loginfo('  [INDICATE_OBJECT]: Look back to object')
    pc.do_relative_angle_change([0,30,30], d, vx, vz)

    rospy.loginfo('  [INDICATE_OBJECT]: Move towards object.')
    rads = pc.get_rpy_of_imu_in_global()
    degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
    pc.do_straight_line(2, degs, d, 0.3, vz)

    rospy.loginfo('  [INDICATE_OBJECT]: Kineme completed!')
    
    return True

def indicate_stay_handler(req):
    d = pc.current_depth
    vx = 0.25
    vz = 0

    # Make a squarish circle by doing 90 degree turns while moving forward, with straight motion in between.
    # TODO: Smooth this out?
    rospy.loginfo('  [INDICATE_STAY]: Kineme initated.')
    rospy.loginfo('  [INDICATE_STAY]: Turn 90 degerees.')
    pc.do_relative_angle_change([0,0,90], d, vx, vz)
    rospy.loginfo('  [INDICATE_STAY]: Now forward for 2 seconds.')
    rads = pc.get_rpy_of_imu_in_global()
    degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
    pc.do_straight_line(2, degs, d, vx, vz)

    rospy.loginfo('  [INDICATE_STAY]: Turn 90 degerees.')
    pc.do_relative_angle_change([0,0,90], d, vx, vz)
    rospy.loginfo('  [INDICATE_STAY]: Now forward for 2 seconds.')
    rads = pc.get_rpy_of_imu_in_global()
    degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
    pc.do_straight_line(2, degs, d, vx, vz)

    rospy.loginfo('  [INDICATE_STAY]: Turn 90 degerees.')
    pc.do_relative_angle_change([0,0,90], d, vx, vz)
    rospy.loginfo('  [INDICATE_STAY]: Now forward for 2 seconds.')
    rads = pc.get_rpy_of_imu_in_global()
    degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
    pc.do_straight_line(2, degs, d, vx, vz)

    rospy.loginfo('  [INDICATE_STAY]: Turn 90 degerees.')
    pc.do_relative_angle_change([0,0,90], d, vx, vz)
    rospy.loginfo('  [INDICATE_STAY]: Now forward for 2 seconds.')
    rads = pc.get_rpy_of_imu_in_global()
    degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
    pc.do_straight_line(2, degs, d, vx, vz)

    rospy.loginfo('  [INDICATE_STAY]: Kineme completed!')

    return True

def lost_handler(req):
    d = pc.current_depth
    vx = 0.2 
    vz = 0

    rospy.loginfo('  [LOST]: Kineme initiated.')
    # Wander the robot forward, looking back and forth. Yaws of 70 degrees
    rospy.loginfo('  [LOST]: Turn to the right.')
    pc.do_relative_angle_change([0, 0, 70], d, vx, vz)
    rospy.loginfo('  [LOST]: Turn to the left.')
    pc.do_relative_angle_change([0, 0, -140], d, vx, vz)
    rospy.loginfo('  [LOST]: Turn to the right.')
    pc.do_relative_angle_change([0, 0,  140], d, vx, vz)
    rospy.loginfo('  [LOST]: Turn to the left.')
    pc.do_relative_angle_change([0, 0, -140], d, vx, vz)
    rospy.loginfo('  [LOST]: Turn to the right (back to center).')
    pc.do_relative_angle_change([0, 0, 70], d, vx, vz)

    rospy.loginfo('  [LOST]: Kineme compelted!')


    return True

def malfunction_handler(req):
    d = pc.current_depth
    vx = 0.2 
    vz = 0.1

    rospy.loginfo('  [MALFUNCTION]: Kineme initiated.')
    # Belly-up the robot (roll 180 deg), then move forwards and up slowly.
    rospy.loginfo('  [MALFUNCTION]: Belly up the robot (roll of 180), moving forward and slightly up.')
    pc.do_relative_angle_change([180, 0, 0], d, vx, vz)
    rads = pc.get_rpy_of_imu_in_global()
    degs = (rads[0] * 180/pi, rads[1] * 180/pi, rads[2] * 180/pi)
    rospy.loginfo('  [MALFUNCTION]: Forward and up more for 2 seconds..')
    pc.do_straight_line(2, degs, d-1, vx, 0.3)

    rospy.loginfo('  [MALFUNCTION]: Kineme completed!')
    
    return True
    

def negative_handler(req):
    d = pc.current_depth
    vx = 0 
    vz = 0

    rospy.loginfo('  [NEGATIVE]: Kineme initiated.')
    # Shake the robot's "head".  Yaws of 15 degrees from center.
    rospy.loginfo('  [NEGATIVE]: Yaw right.')
    pc.do_relative_angle_change([0, 0, 15], d, vx, vz)
    rospy.loginfo('  [NEGATIVE]: Yaw left.')
    pc.do_relative_angle_change([0, 0, -30], d, vx, vz)
    rospy.loginfo('  [NEGATIVE]: Yaw right.')
    pc.do_relative_angle_change([0, 0, 30], d, vx, vz)
    rospy.loginfo('  [NEGATIVE]: Yaw left (back to center).')
    pc.do_relative_angle_change([0, 0, -15], d, vx, vz)

    rospy.loginfo('  [NEGATIVE]: Kineme completed!')
    
    return True
            
def possibly_handler(req):
    d = pc.current_depth
    vx = 0 
    vz = 0

    rospy.loginfo('  [POSSIBLY]: Kineme initiated.')
    # Bobble the robot's "head", Rolls of 15 degrees off center.
    rospy.loginfo('  [POSSIBLY]: Roll clockwise.')
    pc.do_relative_angle_change([ 15, 0, 0], d, vx, vz)
    rospy.loginfo('  [POSSIBLY]: Roll counter clockwise.')
    pc.do_relative_angle_change([-30, 0, 0], d, vx, vz)
    rospy.loginfo('  [POSSIBLY]: Roll clockwise')
    pc.do_relative_angle_change([ 30, 0, 0], d, vx, vz)
    rospy.loginfo('  [POSSIBLY]: Roll counter clockwise (back to center).')
    pc.do_relative_angle_change([-15, 0, 0], d, vx, vz)

    rospy.loginfo('  [POSSIBLY]: Kineme completed!')
    
    return True

def repeat_last_handler(req):
    d = pc.current_depth
    vx = 0 
    vz = 0

    # "Cock an ear" by doing a 20, 10, 15 RPY, then go back after 2 seconds.
    rospy.loginfo('  [REPEAT_LAST]: Kineme initated.')
    rospy.loginfo('  [REPEAT_LAST]: Cock ear.')
    pc.do_relative_angle_change([20, 10, 15], d, vx, vz)
    rospy.loginfo('  [REPEAT_LAST]: Wait.')
    sleep(2)
    rospy.loginfo('  [REPEAT_LAST]: Return to center.')
    pc.do_relative_angle_change([-20, -10, -15], d, vx, vz)

    rospy.loginfo('  [REPEAT_LAST]: Kineme completed.')
    
    return True

def report_battery_handler(req):
    d = pc.current_depth
    vx = 0.2
    vz = 0.25

    rospy.loginfo('  [REPORT_BATTERY]: Kineme initated.')
    rospy.loginfo('  [REPORT_BATTERY]: Loop-de-loop.')
    pc.do_relative_angle_change([0, 180, 0], d, vx, vz)
    rospy.loginfo('  [REPORT_BATTERY]: Part duex.')
    pc.do_relative_angle_change([0, 180, 0], d, vx, vz)
    rospy.loginfo('  [REPORT_BATTERY]: Kineme completed!.')

    return True


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
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep()
        
else:
    pass
