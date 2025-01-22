from cscore import CameraServer
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

front_cam = False
# TODO: change for 2025 lol

layout = robotpy_apriltag.AprilTagFieldLayout().loadField(robotpy_apriltag.AprilTagField.k2024Crescendo)

print(f"number of tags in the layout: {len(layout.getTags())}")

def main():
    with open('/boot/frc.json') as f:
        config = json.load(f)
    camera_info_dict = config['cameras'][0]
    print(f"camera: {camera_info_dict}")
    width = camera_info_dict['width']
    height = camera_info_dict['height']

    nt = ntcore.NetworkTableInstance.getDefault()
    nt.startClient4("wpilibpi2024")
    nt.setServerTeam(2429)
    nt.startDSClient()

    # Table for vision output information TODO: don't use these, just use getEntry.set...()
    smartdash_table = nt.getTable('SmartDashboard')
    counter_publisher = smartdash_table.getIntegerTopic("counter").publish()

    # video_camera.set
    camera = CameraServer.startAutomaticCapture()
    camera.setResolution(640, 360)
    
    input_stream = CameraServer.getVideo()
    output_stream = CameraServer.putVideo('Processed', width, height)

    detector = robotpy_apriltag.AprilTagDetector()
    detector.addFamily("tag36h11")
    
    estimator_config = robotpy_apriltag.AprilTagPoseEstimator.Config(tagSize=0.1651, fx=478, fy=478, cx=640 / 2, cy=360 / 2)  # logitech at 640x360
    pose_estimator = robotpy_apriltag.AprilTagPoseEstimator(estimator_config)

    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(640, 360, 3), dtype=np.uint8)
    # Wait for NetworkTables to start
    time.sleep(0.5)

    counter = 0

    while True:

        start_time = time.time()
        frame_time, input_img = input_stream.grabFrame(img)
        output_img = np.copy(input_img)

        # Notify output of error and skip iteration
        if frame_time == 0:
            output_stream.notifyError(input_stream.getError())
            continue

        # ---------- START ACTUAL IMAGE PROCESSING ----------

        # Convert to HSV and threshold image
        grayscale = cv2.cvtColor(input_img, cv2.COLOR_BGR2GRAY)
        # print(f"shape of grayscale: {grayscale.shape}")
        detections = detector.detect(grayscale)

        # if len(tags) > 0:
        #     for tag in tags:
        #         pose = pose_estimator.estimate(tag)
        #         pose_camera = geo.Transform3d(
        #             geo.Translation3d(pose.x, pose.y, pose.z),
        #             geo.Rotation3d(-pose.rotation().x - np.pi, -pose.rotation().y, pose.rotation().z - np.pi))
        #         pose_nwu = geo.CoordinateSystem.convert(pose_camera, geo.CoordinateSystem.EDN(),
        #                                                 geo.CoordinateSystem.NWU())
        #         # where is camera on robot - origin of frame is center of robot
        #         if front_cam:
        #             # camera_in_robot_frame = geo.Transform3d(geo.Translation3d(0.3, 0, 0.2), geo.Rotation3d(0, 0, 0))  # front of robot
        #             # the camera is in the front and four inches to the left of center
        #             # also looks like a negative value on the y rotation gives the right distance
        #             camera_y_rotation = -30  #  -30 seems to be what makes the distances most accurate statically, but it's not getting the ose right
        #             camera_in_robot_frame = geo.Transform3d(geo.Translation3d(0.3, 0.05, 0.2),geo.Rotation3d(0, math.radians(camera_y_rotation), 0))  # back of robot, rotate up in y?
        #         else:  # camera in back
        #             # camera_in_robot_frame = geo.Transform3d(geo.Translation3d(0.3, 0, 0.2), geo.Rotation3d(0, 0, 0))  # front of robot
        #             camera_y_rotation = -30  # -30 seems to be what makes the distances most accurate statically, but it's not getting the ose right
        #             camera_in_robot_frame = geo.Transform3d(geo.Translation3d(-0.3, -.1, 0.2), geo.Rotation3d(0, math.radians(camera_y_rotation), np.pi))  # back of robot, rotate up in y?
        #
        #         tag_in_field_frame = layout.getTagPose(tag.getId())
        #         try:
        #             robot_in_field_frame = objectToRobotPose(objectInField=tag_in_field_frame, cameraToObject=pose_nwu, robotToCamera=camera_in_robot_frame)
        #             rft = robot_in_field_frame.translation()
        #             rfr = robot_in_field_frame.rotation()
        #             tx, ty, tz = rft.x, rft.y, rft.z
        #             rx, ry, rz = rfr.x, rfr.y, rfr.z
        #             self.tags.update({f'tag{tag.getId():02d}': {'id': tag.getId(), 'rotation': pose.rotation().x, 'distance': pose.z,
        #                                           'tx': tx, 'ty': ty, 'tz': tz, 'rx': rx, 'ry': ry, 'rz': rz}})
        #         except Exception as e:
        #             print(f'Attempted to get field frame but got error {e} on tag id {tag.getId()}')

        # print(f"number of detections: {len(detections)}")

        poses = [pose_estimator.estimate(detection) for detection in detections]

        # list of robot poses as computed from each tag
        robot_poses = []

        for pose in poses:

            # ngl idk why it's good to do these weird rotations but whatever
            pose_camera = geo.Transform3d(
                    geo.Translation3d(pose.x, pose.y, pose.z),
                    geo.Rotation3d(-pose.rotation().x -np.pi, -pose.rotation().y, pose.rotation().z - np.pi)
            )

            # OpenCV and WPILib estimator layout of axes is EDN and field WPILib is NWU; need x -> -y , y -> -z , z -> x and same for differential rotations
            transform_nwu = geo.CoordinateSystem.convert(pose_camera, geo.CoordinateSystem.EDN(), geo.CoordinateSystem.NWU())
            #print(pose_nwu)
            # this may be buggy, according to OP, so try 
            old = geo.CoordinateSystem.EDN()
            new = geo.CoordinateSystem.NWU()

            transform_nwu2 = geo.Transform3d(
                geo.CoordinateSystem.convert(pose_camera.translation(), old, new),
                geo.CoordinateSystem.convert(geo.Rotation3d(), new, old) + (geo.CoordinateSystem.convert(pose_camera.rotation(), old, new)),
            )

            robot_to_camera_transform = geo.Transform3d(
                    geo.Translation3d(0.3, 0.15, 0.2),
                    geo.Rotation3d(0, math.radians(30), 0)
            )

            tag_in_field_frame = layout.getTagPose(7)
            # print(f"tag_in_field_frame: {tag_in_field_frame}")
            if tag_in_field_frame:
                robot_in_field_frame = wpimath.objectToRobotPose(objectInField=tag_in_field_frame, cameraToObject=transform_nwu2, robotToCamera=robot_to_camera_transform)
                robot_poses.append(robot_in_field_frame)


        for idx, pose in enumerate(robot_poses):
            nt.getEntry(f"SmartDashboard/pose {idx} idx x").setFloat(pose.X())
            nt.getEntry(f"SmartDashboard/pose {idx} idx y").setFloat(pose.Y())
            nt.getEntry(f"SmartDashboard/pose {idx} idx z").setFloat(pose.Z())
            nt.getEntry(f"SmartDashboard/pose {idx} idx x rot").setFloat(pose.rotation().x_degrees)
            nt.getEntry(f"SmartDashboard/pose {idx} idx y rot").setFloat(pose.rotation().y_degrees)
            nt.getEntry(f"SmartDashboard/pose {idx} idx z rot").setFloat(pose.rotation().z_degrees)
            # print(f"putting tag {idx} x AND seperately y AND z into smartdash")


        counter += 1
        # print(f"counter is {counter}")
        nt.getEntry("SmartDashboard/counter").setInteger(counter)

        output_stream.putFrame(grayscale)

main()
