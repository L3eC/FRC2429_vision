from typing import List
import numpy as np
import cv2
import robotpy_apriltag
import wpimath.geometry as geo
import wpimath

# stuff which is needed and the same for every camera.
# stuff which changes with different cameras should be passed to estimate_poses_from_apriltag as a parameter.
layout = robotpy_apriltag.AprilTagFieldLayout().loadField(robotpy_apriltag.AprilTagField.k2024Crescendo)
detector = robotpy_apriltag.AprilTagDetector()
detector.addFamily("tag36h11")

def estimate_poses_from_apriltags(image: np.ndarray, pose_estimator: robotpy_apriltag.AprilTagPoseEstimator, robot_to_camera_transform: geo.Transform3d) -> List[geo.Pose2d]:

    """
    Given an image, a pose estimator, and the position of the camera on the robot, returns a list of robot poses, one for each apriltag.
    The function needs a pose estimator because the pose estimator is constructed with lens intrinsics, which change per camera and therefore cannot be hardcoded.
    """

    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    detections = detector.detect(grayscale)

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

        tag_in_field_frame = layout.getTagPose(7)
        # print(f"tag_in_field_frame: {tag_in_field_frame}")
        if tag_in_field_frame:
            robot_in_field_frame = wpimath.objectToRobotPose(objectInField=tag_in_field_frame, cameraToObject=transform_nwu, robotToCamera=robot_to_camera_transform)
            robot_poses.append(robot_in_field_frame)


    # for idx, pose in enumerate(robot_poses):
    #     nt.getEntry(f"SmartDashboard/pose {idx} idx x").setFloat(pose.X())
    #     nt.getEntry(f"SmartDashboard/pose {idx} idx y").setFloat(pose.Y())
    #     nt.getEntry(f"SmartDashboard/pose {idx} idx z").setFloat(pose.Z())
    #     nt.getEntry(f"SmartDashboard/pose {idx} idx x rot").setFloat(pose.rotation().x_degrees)
    #     nt.getEntry(f"SmartDashboard/pose {idx} idx y rot").setFloat(pose.rotation().y_degrees)
    #     nt.getEntry(f"SmartDashboard/pose {idx} idx z rot").setFloat(pose.rotation().z_degrees)

    return robot_poses
