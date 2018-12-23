#! /usr/bin/python

import sys, math, threading
from time import sleep

import rospy

from rcvm_core.srv import *

'''
    Service handlers.
'''

def affirmative_handler(req):
    pass

def attention_handler(req):
    pass

def danger_handler(req):
    pass

def follow_me_handler(req):
    pass

def indicate_movement_handler(req):
    pass

def indicate_object_handler(req):
    pass

def indicate_stay_handler(req):
    pass

def lost_handler(req):
    pass

def malfunction_handler(req):
    pass

def negative_handler(req):
    pass
            
def possibly_handler(req):
    pass

def repeat_last_handler(req):
    pass

def report_battery_handler(req):
    pass




if __name__ == "__main__":
    rospy.init_node('rcvm_server', argv=None, anonymous=True)
    rospy.loginfo('Initializing Aqua8 RCVM server...')

    #With the aircraft version and activation confirmed, we can advertise our services.
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