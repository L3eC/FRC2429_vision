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

def pipeline_handle_apriltags(blockhead_camera: BlockheadCamera) -> Tuple[List[float], Dict[int, float]]:

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
    
    normalized_rightnesses = {}
    for detection in detections:
        normalized_rightness = detections[0].getCenter().x - blockhead_camera.get_frc_json()['width'] / 2 # get it and make 0 the center
        normalized_rightness /= blockhead_camera.get_frc_json()['width'] / 2 # normalize it
        normalized_rightnesses.update({detection.getId(): normalized_rightness})

    ids_poses = {detection.getId(): blockhead_camera.get_pose_estimator().estimate(detection) for detection in detections}

    robot_pose_info = [] # list of robot poses as computed from each tag, with timestamps
                         # each tag takes up 4 items: timestamp, x, y, and rot

    for id, pose in ids_poses.items():

        # ngl idk why we need to do these weird rotations but whatever
        pose_camera = geo.Transform3d(
                geo.Translation3d(pose.x, pose.y, pose.z),
                geo.Rotation3d(-pose.rotation().x -np.pi, -pose.rotation().y, pose.rotation().z - np.pi)
        )

        # OpenCV and WPILib estimator layout of axes is EDN and field WPILib is NWU; need x -> -y , y -> -z , z -> x and same for differential rotations
        transform_nwu = geo.CoordinateSystem.convert(pose_camera, geo.CoordinateSystem.EDN(), geo.CoordinateSystem.NWU())

        tag_in_field_frame = layout.getTagPose(id)

        if tag_in_field_frame:

            robot_in_field_frame = wpimath.objectToRobotPose(objectInField=tag_in_field_frame, cameraToObject=transform_nwu, robotToCamera=robot_to_camera_transform)
            
            this_tag_info = [timestamp, robot_in_field_frame.X(), robot_in_field_frame.Y(), robot_in_field_frame.rotation().Z()]

            robot_pose_info += this_tag_info

    return (robot_pose_info, normalized_rightnesses)
