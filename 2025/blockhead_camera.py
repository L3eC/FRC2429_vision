import math
from cscore import CameraServer, CvSink, UsbCamera
import numpy as np
from robotpy_apriltag import AprilTagPoseEstimate, AprilTagPoseEstimator
from wpimath.geometry import Rotation3d, Transform3d, Translation3d


class BlockheadCamera:

    def __init__(self, frc_json: dict, extra_info_dict_for_this_camera: dict) -> None:

        self.frc_json = frc_json
        self.extra_info_dict_for_this_camera = extra_info_dict_for_this_camera

        self.usb_camera = UsbCamera(frc_json['name'], frc_json['path'])
        # this doesn't seem to happen by default
        self.usb_camera.setResolution(frc_json['width'], frc_json['height'])
        
        self.image_provider = CameraServer.getVideo(self.usb_camera)

        # one for each camera because this depends on lens intrinsics, which differ per camera
        self.apriltag_estimator_config = AprilTagPoseEstimator.Config(
                extra_info_dict_for_this_camera['tag_size'],
                extra_info_dict_for_this_camera['fx'],
                extra_info_dict_for_this_camera['fy'],
                extra_info_dict_for_this_camera['cx'],
                extra_info_dict_for_this_camera['cy'],
        )

        self.apriltag_estimator = AprilTagPoseEstimator(self.apriltag_estimator_config)

        self.robot_to_camera_transform = Transform3d(
                Translation3d(
                    extra_info_dict_for_this_camera['transform']['x_meters'],
                    extra_info_dict_for_this_camera['transform']['y_meters'],
                    extra_info_dict_for_this_camera['transform']['z_meters']
                ),
                Rotation3d(
                    math.radians(extra_info_dict_for_this_camera['transform']['x_rot_degrees']),
                    math.radians(extra_info_dict_for_this_camera['transform']['y_rot_degrees']),
                    math.radians(extra_info_dict_for_this_camera['transform']['z_rot_degrees'])
                )
        )

        num_channels = 3 # red, green, blue
        self.default_frame = np.zeros(shape=(frc_json['width'], frc_json['height'], num_channels), dtype=np.uint8)

    def get_usb_camera(self):
        return self.usb_camera

    def get_frc_json(self):
        return self.frc_json

    def get_extra_info_dict(self):
        return self.extra_info_dict_for_this_camera

    def get_image(self) -> tuple[int, np.ndarray]:
        return self.image_provider.grabFrame(self.default_frame)

    def get_robot_to_camera_transform(self) -> Transform3d:
        return self.robot_to_camera_transform

    def get_pose_estimator(self) -> AprilTagPoseEstimator:
        return self.apriltag_estimator

