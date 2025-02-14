import math
from typing import Dict, List, Tuple
import numpy as np
import cv2
import robotpy_apriltag
import wpimath.geometry as geo
import wpimath
from blockhead_camera import BlockheadCamera

# stuff which is needed and the same for every camera.
# stuff which changes with different cameras should be passed to estimate_poses_from_apriltag as a parameter.
layout = robotpy_apriltag.AprilTagFieldLayout().loadField(robotpy_apriltag.AprilTagField.k2024Crescendo)
detector = robotpy_apriltag.AprilTagDetector()
detector.addFamily("tag36h11")

def pipeline_handle_apriltags(blockhead_camera: BlockheadCamera) -> Tuple[List[float], Dict[int, Tuple[float, float]]]:

    """
    Given a BlockheadCamera, returns a tuple of a list and a dict.
    this list has 4*n floats (n is an integer), 
    where each 4-float chunk represents the robot pose as computed from one tag. 
    Each chunk is of the form [timestamp, robot x, robot y, robot yaw].
    the dictionary has tag ids for keys, and their "rightnesses" for values 
    (where -1 rightness is all the way on the left of the image and 1 is all the way on the right)
    The function needs a pose estimator because the pose 
    estimator is constructed with lens intrinsics, 
    which change per camera and therefore cannot be hardcoded into the function.
    """

    robot_to_camera_transform = blockhead_camera.get_robot_to_camera_transform()

    timestamp, image = blockhead_camera.get_image()

    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    detections = detector.detect(grayscale)

    tag_vector_dict = {} # vectors from camera to tag, stored in polar coords
    robot_pose_info = []

    for detection in detections:

        pose = blockhead_camera.get_pose_estimator().estimate(detection)

        # ngl idk why we need to do these weird rotations but whatever
        pose_camera = geo.Transform3d(
                geo.Translation3d(pose.x, pose.y, pose.z),
                geo.Rotation3d(-pose.rotation().x -np.pi, -pose.rotation().y, pose.rotation().z - np.pi)
        )

        # OpenCV and WPILib estimator layout of axes is EDN and field WPILib is NWU; need x -> -y , y -> -z , z -> x and same for differential rotations
        transform_nwu = geo.CoordinateSystem.convert(pose_camera, geo.CoordinateSystem.EDN(), geo.CoordinateSystem.NWU())

        tag_in_field_frame = layout.getTagPose(detection.getId())

        if tag_in_field_frame:

            robot_in_field_frame = wpimath.objectToRobotPose(objectInField=tag_in_field_frame, cameraToObject=transform_nwu, robotToCamera=robot_to_camera_transform)

            this_tag_info = [timestamp, robot_in_field_frame.X(), robot_in_field_frame.Y(), robot_in_field_frame.rotation().Z()]

            robot_pose_info += this_tag_info

        # cx is the "center" of the image in pixel coords
        zero_centered_x = detection.getCenter().x - blockhead_camera.get_extra_info_dict()['cx']

        # this is the angle between a vector shooting straight out of the camera, and the vector from the camera to the object.
        # as the object moves clockwise, the angle decreases (conforming with the usual system)
        # negative sign in front of zero_centered_x because as the pixel's x coordinate increases, the object moves clockwise relative to us
        # but atan expects an increase in its first argument to result in an increase in the angle.
        tag_angle_from_camera_forward = math.atan2(-zero_centered_x, blockhead_camera.get_extra_info_dict()['fx'])

        distance_to_tag = geo.Translation2d(transform_nwu.x, transform_nwu.y).norm()

        tag_vector_dict.update({detection.getId(): (distance_to_tag, tag_angle_from_camera_forward)})

    return (robot_pose_info, tag_vector_dict)
