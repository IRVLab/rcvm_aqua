#!/usr/bin/env python

import rospy
from aquacore.msg import Command
from rcvm.srv import *


# A head affirmativeding motion, indicating affirmative.
def handle_affirmative(req):
    # Check defined emphasis and set to default if none, return false if invalid.
    emphasis = req.emphasis
    if emphasis == 0 or emphasis == None:
        emphasis == 0.5
    elif emphasis < 0.0 or emphasis > 1.0:
        return AffirmativeResponse(False)
 
    return AffirmativeResponse(True)


# A head shaking motion, indicating no.
def handle_negative(req):
    # Check defined emphasis and set to default if none, return false if invalid.
    emphasis = req.emphasis
    if emphasis == 0 or emphasis == None:
        emphasis == 0.5
    elif emphasis < 0.0 or emphasis > 1.0:
        return NegativeResponse(False)

    return NegativeResponse(True)


# A head possible motion indicating a maybe.
def handle_possible(req):
    # Check defined emphasis and set to default if none, return false if invalid.
    emphasis = req.emphasis
    if emphasis == 0 or emphasis == None:
        emphasis == 0.5
    elif emphasis < 0.0 or emphasis > 1.0:
        return PossibleResponse(False)

    return PossibleResponse(True)

# An analog to the dive signal indicating an ascent by five meters.
def handle_dive_ascend(req):
    return DiveAscendResponse(True)

# An analog to the dive signal indicating a descent by five meters.
def handle_dive_descend(req):
    return DiveDescendResponse(True)

# An analog to the dive signal indicating that we should stay at the current depth for a bit.
def handle_dive_wait(req):
    return DiveWaitResponse(True)

# A motion to catch attention 
def handle_attention(req):
    return AttentionResponse(False)

# A fear-like motion, indicating danger in the area.
def handle_danger(req):
    return DangerResponse(False)

# A beckoning motion, indicating that the collabortor should follow.
def handle_follow_me(req):
    return FollowMeResponse(True)

# A set of motions indicating vision, motor, or other failures.
def handle_malfunction(req):
    return MalfunctionResponse(False)

# A "head cocking" motion asking for a repeat_last of the previous command.
def handle_repeat_last(req):
    return RepeatLastResponse(True)

# A indidcate_objecting motion with the entire body, indicating the direction.
def handle_indidcate_object(req):
    return IndidcateObjectResponse(False)

# A motion indicating current battery state.
def handle_report_energy(req):
    return HandleEnergyResponse(False)

# A confused motion, indicating the loss of localization.
def handle_im_lost(req):
    return ImLostResponse(False)


# The centralized kineme server.
def kineme_server():
    rospy.init_affirmativee('rcvm_server')

    #Initialize all services.

    affirmative = rospy.Service('/rcvm/affirmative', Affirmative, handle_affirmative)
    print "Initialized Affirmative server..."

    negative = rospy.Service('rcvm/negative', Negative, handle_negative)
    print "Initialized Negative server..."

    possible = rospy.Service('rcvm/possible', Possible, handle_possible)
    print "Initialized Possible server..."

    dive_ascend = rospy.Service('/rcvm/dive_ascend', DiveAscend, handle_dive_ascend)
    print "Initialized Dive Ascend server..."

    dive_descend = rospy.Service('/rcvm/dive_descend', DiveDescend, handle_dive_descend)
    print "Initialized Dive Descend server..."

    dive_wait = rospy.Service('/rcvm/dive_wait', DiveWait, handle_dive_wait)
    print "Initialized Dive Stop server..."

    attention = rospy.Service('/rcvm/attention', Attention, handle_attention)
    print "Initialized Attention server..."

    danger = rospy.Service('/rcvm/danger', Danger, handle_danger)
    print "Initialized Danger server..."

    follow_me = rospy.Service('/rcvm/follow_me', FollowMe, handle_follow_me)
    print "Initialized Follow Me server..."

    malfunction = rospy.Service('/rcvm/malfunction', Malfunction, handle_malfunction)
    print "Initialized Malfunction server..."

    repeat_last = rospy.Service('/rcvm/repeat_last', RepeatLast, handle_repeat_last)
    print "Initialized Repeat Last server..."

    indidcate_object = rospy.Service('/rcvm/indidcate_object', IndidcateObject, handle_indidcate_object)
    print "Initialized Indidcate Object server..."

    report_energy = rospy.Service('/rcvm/report_energy', ReportEnergy, handle_report_energy)
    print "Initialized Report Energy server..."

    im_lost = rospy.Service('/rcvm/im_lost', ImLost, handle_im_lost)
    print "Initialized I'm Lost server..."

    rospy.spin()

if __name__ == "__main__":
        rcvm_server()
