#!/usr/bin/env python3
import rospy
from grabbybot_controller.srv import AnglesConverter, AnglesConverterResponse
import math

def convert_radians_to_degrees(req):
    res = AnglesConverterResponse()
    res.base = int(((req.base + math.pi/2)*180)/math.pi)
    res.shoulder = int(180 - ((req.shoulder + math.pi/2)*180)/math.pi)
    res.elbow = int(((req.elbow + math.pi/2)*180)/math.pi)
    res.gripper = int(-((req.gripper + math.pi/2)*180)/math.pi)
    return res

def convert_degrees_to_radians(req):
    res = AnglesConverterResponse()

if __name__ == '__main__':
    rospy.init_node("angles_converter", anonymous=True)
    radians_to_degrees = rospy.Service('radians_to_degrees', AnglesConverter, convert_radians_to_degrees)
    degrees_to_radians = rospy.Service('degrees_to_radians', AnglesConverter, convert_degrees_to_radians)

    rospy.spin()