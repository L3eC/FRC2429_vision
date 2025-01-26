import math
from cscore import CameraServer, CvSink, UsbCamera
import numpy as np
from robotpy_apriltag import AprilTagPoseEstimate, AprilTagPoseEstimator
from wpimath.geometry import Rotation3d, Transform3d, Translation3d


class BlockheadCamera:

    def __init__(self, frc_json: dict, blockhead_json: dict) -> None:

        self.frc_json = frc_json
        self.blockhead_json = blockhead_json

        self.usb_camera = UsbCamera(frc_json['name'], frc_json['path'])
        self.usb_camera.setResolution(frc_json['width'], frc_json['height'])
        
        self.image_provider = CameraServer.getVideo(self.usb_camera)

        self.apriltag_estimator_config = AprilTagPoseEstimator.Config(
                blockhead_json['tag_size'],
                blockhead_json['fx'],
                blockhead_json['fy'],
                blockhead_json['cx'],
                blockhead_json['cy'],
        )

        self.apriltag_estimator = AprilTagPoseEstimator(self.apriltag_estimator_config)

        self.robot_to_camera_transform = Transform3d(
                Translation3d(
                    blockhead_json['transform']['x_meters'],
                    blockhead_json['transform']['y_meters'],
                    blockhead_json['transform']['z_meters']
                ),
                Rotation3d(
                    math.radians(blockhead_json['transform']['x_rot_degrees']),
                    math.radians(blockhead_json['transform']['y_rot_degrees']),
                    math.radians(blockhead_json['transform']['z_rot_degrees'])
                )
        )

        # 3rd dimension length is 3 for 3 channels BGR
        self.default_frame = np.zeros(shape=(frc_json['width'], frc_json['height'], 3), dtype=np.uint8)

    def get_usb_camera(self):
        return self.usb_camera

    def get_frc_json(self):
        return self.frc_json

    def get_blockhead_json(self):
        return self.blockhead_json

    def get_image(self) -> tuple[int, np.ndarray]:
        return self.image_provider.grabFrame(self.default_frame)

    def get_robot_to_camera_transform(self) -> Transform3d:
        return self.robot_to_camera_transform

    def get_pose_estimator(self) -> AprilTagPoseEstimator:
        return self.apriltag_estimator

