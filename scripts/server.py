#!/usr/bin/env python

import rospy
from aquacore.msg import Command
from rcvm.srv import *


def handle_nod(req):
    cycles = req.cycles
    emphasis = req.emphasis

    if cycles == 0:
	cycles = 3
 
    print "Nodding %s times, with an emphasis of %s"%(cycles, emphasis)

    rate = rospy.Rate(60)
    pub = rospy.Publisher('aqua/command', Command, queue_size=10)

    pitches = list()
    for i in xrange(cycles):
	up = [1] * 25
	pitches.extend(up)
	down = [-1] * 30
	pitches.extend(down)

    pitches.extend([1]* 15)

    while not rospy.is_shutdown():
	msg = Command()
	msg.pitch = pitches.pop(0)
#	rospy.loginfo(msg)
	pub.publish(msg)
	rate.sleep()

	if not pitches:
		break

    return NodResponse(True)

def handle_headshake(req):
    cycles = req.cycles
    emphasis = req.emphasis

    if cycles == 0:
	cycles = 3

    print "Shaking head %s times, with an emphasis of %s"%(cycles, emphasis)

    rate = rospy.Rate(60)
    pub = rospy.Publisher('aqua/command', Command, queue_size=10)

    yaws = list()
    for i in xrange(cycles):
	right = [1] * 40
	yaws.extend(right)
	left = [-1] * 50
	yaws.extend(left)

    yaws.extend([1] * 20)

    while not rospy.is_shutdown():
	msg = Command()
	msg.yaw = yaws.pop(0)
#	rospy.loginfo(msg)
	pub.publish(msg)
	rate.sleep()

	if not yaws:
		break

    return HeadshakeResponse(True)

def rcvm_server():
    rospy.init_node('rcvm_server')
    nod = rospy.Service('/rcvm/nod', Nod, handle_nod)
    print "Initialized nod server"

    headshake = rospy.Service('rcvm/headshake', Headshake, handle_headshake)
    print "Initialized headshake server"

    rospy.spin()

if __name__ == "__main__":
        rcvm_server()
