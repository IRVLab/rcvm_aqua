#!/usr/bin/env python

import rospy
from aquacore.msg import Command
from rcvm.srv import *


# A head nodding motion, indicating yes.
def handle_nod(req):
    # Check defined cycles and set to default if none, return false if invalid.
    cycles = req.cycles
    if cycles == 0 or cycles == None:
	    cycles = 2
    elif cycles > 5:
        return NodResponse(False)

    # Check defined emphasis and set to default if none, return false if invalid.
    emphasis = req.emphasis
    if emphasis == 0 or emphasis == None:
        emphasis == 0.5
    elif emphasis < 0.0 or emphasis > 1.0:
        return NodResponse(False)
 
    return NodResponse(True)


# A head shaking motion, indicating no.
def handle_shake(req):
    # Check defined cycles and set to default if none, return false if invalid.
    cycles = req.cycles
    if cycles == 0 or cycles == None:
	    cycles = 2
    elif cycles > 5:
        return NodResponse(False)

    # Check defined emphasis and set to default if none, return false if invalid.
    emphasis = req.emphasis
    if emphasis == 0 or emphasis == None:
        emphasis == 0.5
    elif emphasis < 0.0 or emphasis > 1.0:
        return NodResponse(False)

    return ShakeResponse(True)


# A head bobble motion indicating a maybe.
def handle_bobble(req):
    # Check defined cycles and set to default if none, return false if invalid.
    cycles = req.cycles
    if cycles == 0 or cycles == None:
	    cycles = 2
    elif cycles > 5:
        return NodResponse(False)

    # Check defined emphasis and set to default if none, return false if invalid.
    emphasis = req.emphasis
    if emphasis == 0 or emphasis == None:
        emphasis == 0.5
    elif emphasis < 0.0 or emphasis > 1.0:
        return NodResponse(False)

    return BobbleResponse(True)


#TODO Figure out what this motion should be.
# A motion to catch attention 
def handle_attention(req):

    return AttentionResponse(True)


# A fear-like motion, indicating danger in the area.
def handle_danger(req):

    return DangerResponse(True)nod = rospy.Service('/rcvm/nod', Nod, handle_nod)
    print "Initialized nod server..."


# An analog to the dive signal indicating an ascent by five meters.
def handle_dive_ascend(req):

    return DiveAscendResponse(True)


# An analog to the dive signal indicating a descent by five meters.
def handle_dive_descend(req):

    return DiveDescendResponse(True)


# An analog to the dive signal indicating that we should stay at the current depth for a bit.
def handle_dive_stop(req):

    return DiveStopResponse(True)


# A beckoning motion, indicating that the collabortor should follow.
def handle_follow_me(req):

    return FollowMeResponse(True)


# A confused motion, indicating the loss of localization.
def handle_im_lost(req):

    return ImLostResponse(True)    


# A set of motions indicating vision, motor, or other failures.
def handle_malfunction(req):

    return MalfunctionResponse(True)


# A pointing motion with the entire body, indicating the direction.
def handle_point(req):

    return PointResponse(True)


# A "head cocking" motion asking for a repeat of the previous command.
def handle_repeat(req):

    return RepeatResponse(True)


# A motion indicating current battery state. Depending on battery state, 
# could be a loop-de-loop or a limping motion.
def handle_report_energy(req):

    return HandleEnergyResponse(True)

# A motion indicating that the collabotor should remain in the current location.
def handle_stay_here(req):

    return StayHereResponse(True)


# The centralized kineme server.
def kineme_server():
    rospy.init_node('rcvm_server')

    #Initialize all services.

    nod = rospy.Service('/rcvm/nod', Nod, handle_nod)
    print "Initialized nod server..."

    shake = rospy.Service('rcvm/shake', Shake, handle_shake)
    print "Initialized shake server..."

    bobble = rospy.Service('rcvm/bobble', Bobble, handle_bobble)
    print "Initialized bobble server..."

    attention = rospy.Service('/rcvm/attention', Attention, handle_attention)
    print "Initialized attention server..."

    danger = rospy.Service('/rcvm/danger', Danger, handle_danger)
    print "Initialized danger server..."

    dive_ascend = rospy.Service('/rcvm/dive_ascend', DiveAscend, handle_dive_ascend)
    print "Initialized dive ascend server..."

    dive_descend = rospy.Service('/rcvm/dive_descend', DiveDescend, handle_dive_descend)
    print "Initialized dive descend server..."

    dive_stop = rospy.Service('/rcvm/dive_stop', DiveStop, handle_dive_stop)
    print "Initialized dive stop server..."

    follow_me = rospy.Service('/rcvm/follow_me', FollowMe, handle_follow_me)
    print "Initialized follow me server..."

    im_lost = rospy.Service('/rcvm/im_lost', ImLost, handle_im_lost)
    print "Initialized i'm lost server..."

    malfunction = rospy.Service('/rcvm/malfunction', Malfunction, handle_malfunction)
    print "Initialized malfunction server..."

    point = rospy.Service('/rcvm/point', Point, handle_point)
    print "Initialized point server..."

    repeat = rospy.Service('/rcvm/repeat', Repeat, handle_repeat)
    print "Initialized repeat server..."

    report_energy = rospy.Service('/rcvm/report_energy', ReportEnergy, handle_report_energy)
    print "Initialized report energy server..."

    stay_here = rospy.Service('/rcvm/stay_here', StayHere, handle_stay_here)
    print "Initialized stay here server..."

    rospy.spin()

if __name__ == "__main__":
        rcvm_server()
