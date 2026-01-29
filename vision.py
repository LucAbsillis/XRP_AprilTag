#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#


import ntcore
import robotpy_apriltag
from cscore import CameraServer,MjpegServer,UsbCamera
import math
import cv2
import numpy as np


#
# This code will work both on a RoboRIO and on other platforms. The exact mechanism
# to run it differs depending on whether you’re on a RoboRIO or a coprocessor
#
# https://robotpy.readthedocs.io/en/stable/vision/code.html


def main():
    # Get the default NetworkTables instance
    inst = ntcore.NetworkTableInstance.getDefault()

    # Start a NetworkTables 4 client and connect to the local simulator
    inst.startClient4("VisionClient")
    inst.setServer("localhost") # In simulation, the server is on your own PC


    detector = robotpy_apriltag.AprilTagDetector()

    # Look for tag36h11, correct 3 error bits
    detector.addFamily("tag36h11", 3)

    # Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
    # (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
    # poseEstConfig = robotpy_apriltag.AprilTagPoseEstimator.Config(
    #     0.1651,
    #     699.3778103158814,
    #     677.7161226393544,
    #     345.6059345433618,
    #     207.12741326228522,
    # )
    poseEstConfig = robotpy_apriltag.AprilTagPoseEstimator.Config(
        0.1651,
        1400,
        1400,
        960,
        540,
    )
    estimator = robotpy_apriltag.AprilTagPoseEstimator(poseEstConfig)
    #server = CameraServer.addServer("First Camera")
    #camera = UsbCamera("First Camera", 0)
    #server.setSource(camera)
    # Get the UsbCamera from CameraServer
    camera = CameraServer.startAutomaticCapture(1)
    # get camera 1
    #camera = CameraServer.startAutomaticCapture()

    # Set the resolution
    camera.setResolution(1920, 1080)

    # Get a CvSink. This will capture Mats from the camera
    cvSink = CameraServer.getVideo()

    # Set up a CvSource. This will send images back to the Dashboard
    #
    outputStream = CameraServer.putVideo("Detected", 1920, 1080)

    # Mats are very memory expensive. Let's reuse these.
    mat = np.zeros((1080, 1920, 3), dtype=np.uint8)
    grayMat = np.zeros(shape=(1080, 1920), dtype=np.uint8)
    radToDegree = 180/math.pi
    # Instantiate once
    tags = []  # The list where the tags will be stored
    outlineColor = (0, 255, 0)  # Color of Tag Outline
    crossColor = (0, 0, 25)  # Color of Cross

    # Output the list to Network Tables
    tagsTable = ntcore.NetworkTableInstance.getDefault().getTable("apriltags")
    pubTags = tagsTable.getIntegerArrayTopic("tags").publish()
    print("pub tags:",pubTags)
    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source mat.  If there is an error notify the output.
        if cvSink.grabFrame(mat) == 0:
            # Send the output frame the error
            outputStream.notifyError(cvSink.getError())
            #print("cvSink grabframe error")
            # Skip the rest of the current iteration
            continue

        cv2.cvtColor(mat, cv2.COLOR_RGB2GRAY, dst=grayMat)

        detections = detector.detect(grayMat)


        tags.clear()

        for detection in detections:
            # Remember the tag we saw
            tags.append(detection.getId())
            #print("detectionID:detection.getId: ",detection.getId())
            # Draw lines around the tag
            for i in range(4):
                j = (i + 1) % 4
                point1 = (int(detection.getCorner(i).x), int(detection.getCorner(i).y))
                point2 = (int(detection.getCorner(j).x), int(detection.getCorner(j).y))
                mat = cv2.line(mat, point1, point2, outlineColor, 2)

            # Mark the center of the tag
            cx = int(detection.getCenter().x)
            cy = int(detection.getCenter().y)
            ll = 10
            mat = cv2.line(
                mat,
                (cx - ll, cy),
                (cx + ll, cy),
                crossColor,
                2,
            )
            mat = cv2.line(
                mat,
                (cx, cy - ll),
                (cx, cy + ll),
                crossColor,
                2,
            )

            # Identify the tag
            mat = cv2.putText(
                mat,
                str(detection.getId()),
                (cx + ll, cy),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                crossColor,
                3,
            )

            # Determine Tag Pose
            pose = estimator.estimate(detection)
            rot = pose.rotation()
            translation = pose.translation()
            bearing = math.atan2(translation.X(),translation.Z())*radToDegree
            stop=1
            # put pose into dashboard

            # set calculated distance(meters) and z axis angle (in radians) 
            #print("Distance: ",pose.translation().norm()," Angle: ", pose.rotation().X())
            
            #tagsTable.getEntry(f"aprilID_{detection.getId()}").setDoubleArray(
             #   [pose.translation().norm(), pose.rotation().X()]
            tagsTable.getEntry(f"aprilID_{detection.getId()}").setDoubleArray(
                [round(pose.Z(),2), round(bearing,2)]
#                [round(pose.X(),2), round(pose.Y(),2), round(pose.Z(),2), round(rot.X(),4), round(rot.Y(),4), round(rot.Z(),4)]
            )

        # Put List of Tags onto Dashboard
        pubTags.set(tags)

        # Give output stream a new image to display
        outputStream.putFrame(mat)

    # The camera code will be killed when the robot.py program exits. If you wish to perform cleanup,
    # you should register an atexit handler. The child process will NOT be launched when running the robot code in
    # simulation or unit testing mode
#if __name__ == "__main__":
#   main()
