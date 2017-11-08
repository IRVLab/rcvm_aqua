#!/usr/bin/env python

import rospy
import aquacore
from rcvm.srv import *

def handle_nod(req):
    print "Nodding %s times, with an emphasis of %s"%(req.cycles, req.emphasis)
    return NodResponse(True)

def handle_headshake(req):
    print "Shaking head %s times, with an emphasis of %s"%(req.cycles, req.emphasis)
    return HeadshakeResponse(True)

def rcvm_server():
    rospy.init_node('rcvm_server')
    nod = rospy.Service('nod_service', Nod, handle_nod)
    print "Initialized nod server"

    headshake = rospy.Service('headshake_service', Headshake, handle_headshake)
    print "Initialized headshake server"
    rospy.spin()

if __name__ == "__main__":
        rcvm_server()
