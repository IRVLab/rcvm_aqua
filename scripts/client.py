#!/usr/bin/env python

import rospy
from rcvm.srv import *

def nod_client(cycles, emphasis):
    rospy.wait_for_service('rcvm/nod')
    try:
        nod = rospy.ServiceProxy('rcvm/nod', Nod)
        resp = nod(cycles, emphasis)
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def headshake_client(cycles, emphasis):
    rospy.wait_for_service('rcvm/headshake')
    try:
        headshake = rospy.ServiceProxy('rcvm/headshake', Headshake)
        resp = headshake(cycles, emphasis)
        print resp
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    nod_client(4, 0.6)
    headshake_client(4, 0.1)
