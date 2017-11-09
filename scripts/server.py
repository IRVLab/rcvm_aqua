#!/usr/bin/env python

import rospy
from aquacore.msg import Command
from rcvm.srv import *


def handle_nod(req):
    print "Nodding %s times, with an emphasis of %s"%(req.cycles, req.emphasis)

    rate = rospy.Rate(10)
    pub = rospy.Publisher('aqua/command', Command, queue_size=10)

    pitches = list()
    for i in xrange(req.cycles):
	pitches.extend([0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.75, -0.75, -0.75, -0.75, -0.75, -0.75, -0.75, -0.75, -0.75, -0.75, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1])

    while not rospy.is_shutdown():
	msg = Command()
	msg.pitch = pitches.pop(0)
	rospy.loginfo(msg)
	pub.publish(msg)
	rate.sleep()

	if not pitches:
		break

    return NodResponse(True)

def handle_headshake(req):
    print "Shaking head %s times, with an emphasis of %s"%(req.cycles, req.emphasis)

    rate = rospy.Rate(10)
    pub = rospy.Publisher('aqua/command', Command, queue_size=10)

    yaws = list()
    for i in xrange(req.cycles):
	yaws.extend([0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75, 0.75,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -0.75, -0.75, -0.75, -0.75, -0.75, -0.75, -0.75, -0.75, -0.75, -0.75, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1])

    while not rospy.is_shutdown():
	msg = Command()
	msg.yaw = yaws.pop(0)
	rospy.loginfo(msg)
	pub.publish(msg)
	rate.sleep()

	if not yaws:
		break

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
