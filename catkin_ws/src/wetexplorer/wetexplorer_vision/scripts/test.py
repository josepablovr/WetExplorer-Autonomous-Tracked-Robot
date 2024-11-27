#!/catkin_ws/src/wetexplorer/wetexplorer_vision/env/bin/python

import argparse
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import utils


def run(model: str, width: int, height: int, num_threads: int, enable_edgetpu: bool) -> None:
    """Continuously run inference on images acquired from the RealSense camera.

    Args:
      model: Name of the TFLite object detection model.
      width: The width of the frame captured from the camera.
      height: The height of the frame captured from the camera.
      num_threads: The number of CPU threads to run the model.
      enable_edgetpu: True/False whether the model is a EdgeTPU model.
    """

    # Configure RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, 30)

    # Start the RealSense pipeline
    pipeline.start(config)

    # Variables to calculate FPS
    counter, fps = 0, 0
    start_time = time.time()

    # Visualization parameters
    row_size = 20  # pixels
    left_margin = 24  # pixels
    text_color = (0, 0, 255)  # red
    font_size = 1
    font_thickness = 1
    fps_avg_frame_count = 10

    # Initialize the object detection model
    base_options = core.BaseOptions(
        file_name=model, use_coral=enable_edgetpu, num_threads=num_threads)
    detection_options = processor.DetectionOptions(
        max_results=10, score_threshold=0.7)
    options = vision.ObjectDetectorOptions(
        base_options=base_options, detection_options=detection_options)
    detector = vision.ObjectDetector.create_from_options(options)

    try:
        while True:
            # Wait for a new frame from the RealSense camera
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            # Convert the frame to a numpy array
            image = np.asanyarray(color_frame.get_data())
            counter += 1

            # Convert the image from BGR to RGB as required by the TFLite model.
            rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            # Create a TensorImage object from the RGB image.
            input_tensor = vision.TensorImage.create_from_array(rgb_image)

            # Run object detection estimation using the model.
            detection_result = detector.detect(input_tensor)

            # Draw keypoints and edges on the input image
            image = utils.visualize(image, detection_result)

            # Calculate the FPS
            if counter % fps_avg_frame_count == 0:
                end_time = time.time()
                fps = fps_avg_frame_count / (end_time - start_time)
                start_time = time.time()

            # Show the FPS
            fps_text = 'FPS = {:.1f}'.format(fps)
            text_location = (left_margin, row_size)
            cv2.putText(image, fps_text, text_location, cv2.FONT_HERSHEY_PLAIN,
                        font_size, text_color, font_thickness)

            # Stop the program if the ESC key is pressed.
            if cv2.waitKey(1) == 27:
                break
            cv2.imshow('object_detector', image)

    finally:
        # Stop the RealSense pipeline and close OpenCV windows
        pipeline.stop()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument(
        '--model',
        help='Path of the object detection model.',
        required=False,
        default='ring2.tflite')
    parser.add_argument(
        '--frameWidth',
        help='Width of frame to capture from camera.',
        required=False,
        type=int,
        default=640)
    parser.add_argument(
        '--frameHeight',
        help='Height of frame to capture from camera.',
        required=False,
        type=int,
        default=480)
    parser.add_argument(
        '--numThreads',
        help='Number of CPU threads to run the model.',
        required=False,
        type=int,
        default=4)
    parser.add_argument(
        '--enableEdgeTPU',
        help='Whether to run the model on EdgeTPU.',
        action='store_true',
        required=False,
        default=False)
    args = parser.parse_args()

    run(args.model, args.frameWidth, args.frameHeight, args.numThreads, args.enableEdgeTPU)


if __name__ == '__main__':
    main()