#!/usr/bin/env python

import rospy
from rcvm.srv import *

def nod_client(cycles, emphasis):
    rospy.wait_for_service('nod_service')
    try:
        nod = rospy.ServiceProxy('nod_service', Nod)
        resp = nod(cycles, emphasis)
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def headshake_client(cycles, emphasis):
    rospy.wait_for_service('headshake_service')
    try:
        headshake = rospy.ServiceProxy('headshake_service', Headshake)
        resp = headshake(cycles, emphasis)
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    nod_client(2, 0.6)
    headshake_client(4, 0.1)
