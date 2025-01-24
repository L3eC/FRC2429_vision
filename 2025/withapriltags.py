from cscore import CameraServer, UsbCamera
import cscore
from cv2.gapi import video
import ntcore
import math
import cv2
import json
import numpy as np
import time
import robotpy_apriltag
import wpimath
import wpimath.geometry as geo
from estimate_poses_from_apriltags import estimate_poses_from_apriltags

front_cam = False
# TODO: change for 2025 lol

layout = robotpy_apriltag.AprilTagFieldLayout().loadField(robotpy_apriltag.AprilTagField.k2024Crescendo)

print(f"number of tags in the layout: {len(layout.getTags())}")

def main():


    nt = ntcore.NetworkTableInstance.getDefault()
    nt.startClient4("wpilibpi2024")
    nt.setServerTeam(2429)
    nt.startDSClient()

    with open('/boot/frc.json') as f:
        config = json.load(f)
    camera_info_dict = config['cameras'][1]
    print(f"camera: {camera_info_dict}")
    width = camera_info_dict['width']
    height = camera_info_dict['height']


    # --------- per camera stuff here ----------
    # not sure this is the best way- maybe make a class or a dictionary?
    # but that's less explicit and more difficult to understand; and this whole rewrite is to
    # make the code easier to understand, and we'll have to type all this anyways.
    # but this is more annoying to change per pi
    # i think we should have different programs per pi anyways so we don't need to know the state of our "main"
    # program before uploading
    # LHACK 1/23/25

    # if you want to see a specific camera on wpilibpi.local:1181, comment out everything relating to other cameras
    # im sure there's a less janky way of doing it but idk what it is

    # at least we should probably populate this stuff from the frc.json

    c920 = UsbCamera("rear logitech c920", "/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_EA3BCE4F-video-index0")
    c920_width = 640
    c920_height = 360
    c920.setResolution(c920_width, c920_height)
    c920_image_provider = CameraServer.getVideo(c920)
    c920_estimator_config = robotpy_apriltag.AprilTagPoseEstimator.Config(tagSize=0.1651, fx=478, fy=478, cx=640 / 2, cy=360 / 2)  # logitech at 640x360
    c920_pose_estimator = robotpy_apriltag.AprilTagPoseEstimator(c920_estimator_config)
    c920_robot_to_camera_transform = geo.Transform3d(
            geo.Translation3d(0.3, 0.15, 0.2),
            geo.Rotation3d(0, math.radians(30), 0)
    )


    lifecam = UsbCamera("test lifecam", "/dev/v4l/by-id/usb-Microsoft_Microsoft®_LifeCam_HD-3000-video-index0")
    lifecam_width = 640
    lifecam_height = 360
    lifecam.setResolution(lifecam_width, lifecam_height)
    lifecam_image_provider = CameraServer.getVideo(lifecam)
    lifecam_estimator_config = robotpy_apriltag.AprilTagPoseEstimator.Config(tagSize=0.1651, fx=478, fy=478, cx=640 / 2, cy=360 / 2)  # logitech at 640x360
    lifecam_pose_estimator = robotpy_apriltag.AprilTagPoseEstimator(lifecam_estimator_config)
    lifecam_robot_to_camera_transform = geo.Transform3d(
            geo.Translation3d(0, 0.3, 0.2),
            geo.Rotation3d(0, 0, math.radians(-90))
    )


    # you get the adjustment sliders etc. on wpilibpi.local:1181 for the camera you call this on
    CameraServer.startAutomaticCapture(lifecam)
    

    output_stream = CameraServer.putVideo('Processed', lifecam_width, lifecam_height)



    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(640, 360, 3), dtype=np.uint8)
    # Wait for NetworkTables to start
    time.sleep(0.5)

    counter = 0

    while True:

        start_time = time.time()

        robot_poses = []

        c920_timestamp, c920_image = c920_image_provider.grabFrame(img)
        robot_poses += estimate_poses_from_apriltags(c920_image, c920_pose_estimator, c920_robot_to_camera_transform)

        if c920_timestamp == 0:
            output_stream.notifyError(c920_image_provider.getError())
            continue


        lifecam_timestamp, lifecam_image = lifecam_image_provider.grabFrame(img)
        robot_poses += estimate_poses_from_apriltags(lifecam_image, lifecam_pose_estimator, lifecam_robot_to_camera_transform)

        if lifecam_timestamp == 0:
            output_stream.notifyError(lifecam_image_provider.getError())

        output_img = np.copy(lifecam_image)

        # Notify output of error and skip iteration


        # ---------- START ACTUAL IMAGE PROCESSING ----------


        for idx, pose in enumerate(robot_poses):
            nt.getEntry(f"SmartDashboard/pose {idx} idx x").setFloat(pose.X())
            nt.getEntry(f"SmartDashboard/pose {idx} idx y").setFloat(pose.Y())
            nt.getEntry(f"SmartDashboard/pose {idx} idx z").setFloat(pose.Z())
            nt.getEntry(f"SmartDashboard/pose {idx} idx rot").setFloat(pose.rotation().x_degrees)
            nt.getEntry(f"SmartDashboard/pose {idx} idx rot").setFloat(pose.rotation().y_degrees)
            nt.getEntry(f"SmartDashboard/pose {idx} idx rot").setFloat(pose.rotation().z_degrees)
            # print(f"putting tag {idx} x AND seperately y AND z into smartdash")


        # counter += 1
        # # print(f"counter is {counter}")
        # nt.getEntry("SmartDashboard/counter").setInteger(counter)

        output_stream.putFrame(lifecam_image)

main()
