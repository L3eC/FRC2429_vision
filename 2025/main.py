import numpy as np
from cscore import CameraServer
import ntcore
print('importing json!')
import json
import time
import robotpy_apriltag
from pipelines.find_robot_pose_from_apriltags_pipeline import find_robot_pose_from_apriltags_pipeline
from pipelines.annotate_image_pipeline import annotate_image_pipeline
from blockhead_camera import BlockheadCamera

# TODO: change for 2025 lol
layout = robotpy_apriltag.AprilTagFieldLayout().loadField(robotpy_apriltag.AprilTagField.k2024Crescendo)

def main():

    print('main started successfully')

    nt = ntcore.NetworkTableInstance.getDefault()
    nt.startClient4("wpilibpi2024")
    nt.setServerTeam(2429)
    nt.startDSClient()

    with open('/boot/frc.json') as f:
        config = json.load(f) # a config provided by wpilib tools, basically.

    with open('practicebot_top_pi_blockhead.json') as j:
        blockhead_camera_jsons = json.load(j) # a config file where we specify certain things for each camera. 
                                              # we don't put this in the main config because the main config is annoying to edit.

    # dictionary to hold BlockheadCameras (see the class)
    blockhead_cameras = {}

    # populate the dictionary
    for frc_json in config['cameras']:
        blockhead_cameras.update({frc_json['name']: BlockheadCamera(frc_json=frc_json, blockhead_json=blockhead_camera_jsons[frc_json['name']])})

    # you get the adjustment sliders etc. on wpilibpi.local:1181 for the UsbCamera you call this on
    CameraServer.startAutomaticCapture(blockhead_cameras['rear logitech c920'].get_usb_camera())
    
    # idk what we want to send as the output
    # maybe we make an annotation pipeline that annotates then returns the image from some camera
    # then use that
    output_width = blockhead_cameras['rear logitech c920'].get_frc_json()['width']
    output_height = blockhead_cameras['rear logitech c920'].get_frc_json()['height']
    output_stream = CameraServer.putVideo('Processed', output_width, output_height)
    output_image = np.zeros(shape=(output_width, output_height), dtype=np.uint8)

    # Wait for NetworkTables to start
    time.sleep(0.5)

    while True:

        start_time = time.time()

        # ---------- START ACTUAL IMAGE PROCESSING ----------

        robot_poses = []

        for blockhead_camera in blockhead_cameras.values(): # run the proper pipelines for each camera

            # we pass a BlockheadCamera to all our pipelines. However, we only want to use some pipelines with some cameras.
            # Therefore, in each camera's blockhead_json entry, we define all the pipelines we want to pass that camera to.
            # We define these pipelines in the below list.
            this_camera_pipeline_names = blockhead_camera.get_blockhead_json()['pipelines']

            # Pass this camera to every pipeline we want it to be passed to.
            if 'find_robot_pose_from_apriltags_pipeline' in this_camera_pipeline_names:
                robot_poses += find_robot_pose_from_apriltags_pipeline(blockhead_camera)

            if 'annotate_image_pipeline' in this_camera_pipeline_names:
                output_image = annotate_image_pipeline(blockhead_camera)

        for idx, pose in enumerate(robot_poses):
            nt.getEntry(f"SmartDashboard/pose {idx} idx x").setFloat(pose.X())
            nt.getEntry(f"SmartDashboard/pose {idx} idx y").setFloat(pose.Y())
            nt.getEntry(f"SmartDashboard/pose {idx} idx z").setFloat(pose.Z())
            nt.getEntry(f"SmartDashboard/pose {idx} idx rot").setFloat(pose.rotation().x_degrees)
            nt.getEntry(f"SmartDashboard/pose {idx} idx rot").setFloat(pose.rotation().y_degrees)
            nt.getEntry(f"SmartDashboard/pose {idx} idx rot").setFloat(pose.rotation().z_degrees)
            # print(f"putting tag {idx} x AND seperately y AND z into smartdash")

        output_stream.putFrame(output_image)

main()
