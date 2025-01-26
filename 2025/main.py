from typing import Dict
import numpy as np
from cscore import CameraServer
import ntcore
import json
import time
from pipelines.find_robot_pose_from_apriltags_pipeline import find_robot_pose_from_apriltags_pipeline
from pipelines.annotate_image_pipeline import annotate_image_pipeline
from blockhead_camera import BlockheadCamera

def main():

    print('main started successfully')

    # start networktables
    nt = ntcore.NetworkTableInstance.getDefault()
    nt.startClient4("wpilibpi2024")
    nt.setServerTeam(2429)
    nt.startDSClient()

    with open('2024compfrc.json') as f:
        frc_json = json.load(f) # a config provided by wpilib tools, basically. Includes stuff necessary for interfacing with the camera itself, as well as camera settings.

    with open('extra_camera_info.json') as j:
        extra_camera_info_json = json.load(j) # a config file where we specify our own things for each camera. 
                                              # we don't merge this with the main config because the main config is annoying to edit.

    # dictionary to hold BlockheadCameras (see the class)
    blockhead_cameras: Dict[str, BlockheadCamera] = {}

    # populate the dictionary with BlockheadCameras made from frc.json entries and the corresponding extra_camera_info.json entries
    # note that each raspberry pi will only have its attached cameras in its frc.json. Therefore, each pi will only look at the portions of
    # extra_camera_info relevant to itself.
    
    # we find the proper extra_camera_info entry based off the camera's name as specified in frc.json
    for frc_json_camera_entry in frc_json['cameras']:
        blockhead_cameras.update({frc_json_camera_entry['name']: BlockheadCamera(frc_json=frc_json_camera_entry, 
                                                                                 extra_info_dict_for_this_camera=extra_camera_info_json[frc_json_camera_entry['name']])}) 


    # you get the adjustment sliders etc. on wpilibpi.local:1181 for the UsbCamera you call this on
    CameraServer.startAutomaticCapture(blockhead_cameras['rear logitech c920'].get_usb_camera())
    
    # idk what we want to send as the output
    # for now we use a a placeholder pipeline that returns an annotated image
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
            this_camera_pipeline_names = blockhead_camera.get_extra_info_dict()['pipelines']

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

        output_stream.putFrame(output_image)

main()
