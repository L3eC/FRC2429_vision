import numpy as np
import cv2
from blockhead_camera import BlockheadCamera

def annotate_image_pipeline(blockhead_camera: BlockheadCamera) -> np.ndarray:
    timestamp, raw_image = blockhead_camera.get_image()
    grayscale = cv2.cvtColor(raw_image, cv2.COLOR_BGR2GRAY)
    processed_image = cv2.putText(grayscale, "this image has been processed!", (40, 40), 1, 2, (255, 0, 0))
    return processed_image

