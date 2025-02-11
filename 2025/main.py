import os
from typing import Dict
import numpy as np
from cscore import CameraServer
import ntcore
import json
import time

from blockhead_camera import BlockheadCamera

from pipeline_handle_apriltags import pipeline_handle_apriltags
from pipeline_annotate_image import pipeline_annotate_image

def main():

    print('main started successfully')

    with open('/boot/frc.json') as f:
        frc_json = json.load(f) # a config provided by wpilib tools, basically. Includes stuff necessary for interfacing with the camera itself, as well as camera settings.

    with open('extra_camera_info.json') as j:
        extra_camera_info_json = json.load(j) # a config file where we specify our own things for each camera. 

    found_pi_info = False
    filenames_in_home_directory = os.listdir('/home/pi')
    for filename in filenames_in_home_directory:
        if 'extra_pi_info' in filename:
            with open(filename) as k:
                extra_pi_info_json = json.load(k)
                found_pi_info = True
                this_pi_name = extra_pi_info_json["name"] # we need this for nt

    if not found_pi_info:
        raise FileNotFoundError("Could not find info for this pi!")

    # start networktables
    nt = ntcore.NetworkTableInstance.getDefault()
    nt.startClient4(this_pi_name)
    nt.setServerTeam(2429)
    nt.startDSClient()

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
    # TODO: try calling this on all our blockheadcameras and seeing if they show up on wpilibpi.local/118n where n > 1
    CameraServer.startAutomaticCapture(blockhead_cameras['rear logitech c920'].get_usb_camera())
    
    # idk what we want to send as the output
    # for now we use a a placeholder pipeline that returns an annotated image
    output_width = blockhead_cameras['rear logitech c920'].get_frc_json()['width']
    output_height = blockhead_cameras['rear logitech c920'].get_frc_json()['height']
    # TODO: try making several output streams and seeing if they go onto wpilibpi.local/118n where n > 1
    output_stream = CameraServer.putVideo('Processed', output_width, output_height)
    output_image = np.zeros(shape=(output_width, output_height), dtype=np.uint8)

    # Wait for NetworkTables to start
    time.sleep(0.5)

    while True:

        start_time = time.time()

        # ---------- START ACTUAL IMAGE PROCESSING ----------

        robot_pose_info = []
        tag_rightnesses = {}

        for blockhead_camera in blockhead_cameras.values(): # run the proper pipelines for each camera

            # we pass a BlockheadCamera to all our pipelines. However, we only want to use some pipelines with some cameras.
            # Therefore, in each camera's blockhead_json entry, we define all the pipelines we want to pass that camera to.
            this_camera_pipeline_names = blockhead_camera.get_extra_info_dict()['pipelines']

            # Pass this camera to every pipeline we want it to be passed to.
            if 'pipeline_find_robot_pose_from_apriltags' in this_camera_pipeline_names:

                this_camera_robot_pose_info, this_camera_rightnesses = pipeline_handle_apriltags(blockhead_camera)

                robot_pose_info += this_camera_robot_pose_info
                tag_rightnesses.update(this_camera_rightnesses)

            if 'pipeline_annotate_image' in this_camera_pipeline_names:
                output_image = pipeline_annotate_image(blockhead_camera)

        # ------------------- TRANSMIT RESULTS ----------------

        nt.getEntry(f"vision/{this_pi_name}/robot_pose_info").setFloatArray(robot_pose_info)

        for id, rightness in tag_rightnesses.items():
            nt.getEntry(f"vision/{this_pi_name}/rightnesses/id {id}").setFloat(rightness)

        output_stream.putFrame(output_image)

main()
