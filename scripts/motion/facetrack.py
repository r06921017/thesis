#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Use Tracking Module to Track a Face"""

import qi
import argparse
import sys
import rospy


if __name__ == "__main__":
    print 'face track start!'
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.50.99",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559, help="Naoqi port number")
    parser.add_argument("--facesize", type=float, default=0.1, help="Face width.")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) + ".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    motion_service = session.service("ALMotion")
    tracker_service = session.service("ALTracker")
    rest_flag = False

    # First, wake up.
    # motion_service.wakeUp()

    # Add target to track.
    target_name = "Face"

    while not rospy.is_shutdown():
        if rospy.get_param('/thesis/face_track', False):
            tracker_service.registerTarget(target_name, args.facesize)
            # Then, start tracker. 
            tracker_service.track(target_name) 

        else:
            tracker_service.stopTracker()
            tracker_service.unregisterAllTargets()

        rospy.sleep(0.5)
