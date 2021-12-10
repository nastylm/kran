#The main file

from time import time, strftime
import v4l2
from Zoomm import *


# sys.path.append('/opt/ros/indigo/lib/python2.7/dist-packages')
# need to run the following line before running the script in ros mode
# source /opt/ros/indigo/setup.bash

# python imports
import signal
import cv2
import math
import numpy as np

# application imports
from PerspectiveTransform import PerspectiveCorrecter
from MarkerPose import MarkerPose
from MarkerTracker import MarkerTracker

# parameters
print_debug_messages = False
show_image = True
list_of_markers_to_find = [4]
get_images_to_flush_cam_buffer = 5
publish_to_ros = False
markerpose_ros_topic = '/markerlocator/markerpose'

# global variables
stop_flag = False


# define ctrl-c handler
def signal_handler(signal, frame):
    global stop_flag
    stop_flag = True

# install
signal.signal(signal.SIGINT, signal_handler)

class CameraDriver:

    def __init__(self, marker_orders=[4], default_kernel_size=21, scaling_parameter=2500, downscale_factor=1):
        # Initialize camera driver.
        # Open output window.
        if show_image is True:
            cv2.namedWindow('filterdemo', cv2.WINDOW_AUTOSIZE)

        # Select the camera where the images should be grabbed from.
        #self.camera = cv2.VideoCapture('rtsp://admin:EZtieyS15ef05@192.168.1.64/1')
        self.camera = cv2.VideoCapture(0)
        self.set_camera_resolution()

        # Storage for image processing.
        self.current_frame = None
        self.processed_frame = None
        self.running = True
        self.downscale_factor = downscale_factor


        # Storage for trackers.
        self.trackers = []
        self.old_locations = []

        # Initialize trackers.
        for marker_order in marker_orders:
            temp = MarkerTracker(marker_order, default_kernel_size, scaling_parameter)
            temp.track_marker_with_missing_black_leg = False
            self.trackers.append(temp)
            self.old_locations.append(MarkerPose(None, None, None, None, None))

    def set_camera_resolution(self):
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    def get_image(self):
        # Get image from camera.
        for k in range(get_images_to_flush_cam_buffer):
            self.current_frame = self.camera.read()[1]

    def process_frame(self):
        self.processed_frame = self.current_frame
        # Locate all markers in image.
        frame_gray = cv2.cvtColor(self.current_frame, cv2.COLOR_RGB2GRAY)
        reduced_image = cv2.resize(frame_gray, (0, 0), fx=1.0/self.downscale_factor, fy=1.0 / self.downscale_factor)
        for k in range(len(self.trackers)):
            # Previous marker location is unknown, search in the entire image.
            self.current_frame = self.trackers[k].locate_marker(reduced_image)
            self.old_locations[k] = self.trackers[k].pose
            self.old_locations[k].scale_position(self.downscale_factor)

    def draw_detected_markers(self):
        for k in range(len(self.trackers)):
            xm = self.old_locations[k].x
            ym = self.old_locations[k].y
            orientation = self.old_locations[k].theta
            if self.old_locations[k].quality < 0.9:
                cv2.circle(self.processed_frame, (xm, ym), 4, (55, 55, 255), 1)
            else:
                cv2.circle(self.processed_frame, (xm, ym), 4, (55, 55, 255), 3)

            xm2 = int(xm + 50 * math.cos(orientation))
            ym2 = int(ym + 50 * math.sin(orientation))
            cv2.line(self.processed_frame, (xm, ym), (xm2, ym2), (255, 0, 0), 2)
            #cv2.putText(self.processed_frame, "%8.3f %8.3f %8.3f %8.3f %s", (10, 700), 5, 0.9, (0, 255, 0))

    def show_processed_frame(self):
        if show_image is True:
            cv2.imshow('filterdemo', self.processed_frame)


    def reset_all_locations(self):
        # Reset all markers locations, forcing a full search on the next iteration.
        for k in range(len(self.trackers)):
            self.old_locations[k] = MarkerPose(None, None, None, None, None)

    def handle_keyboard_events(self):
        if show_image is True:
            # Listen for keyboard events and take relevant actions.
            key = cv2.waitKey(100)
            # Discard higher order bit, http://permalink.gmane.org/gmane.comp.lib.opencv.devel/410
            key = key & 0xff
            if key == 27:  # Esc
                self.running = False
            if key == 114:  # R
                print("Resetting")
                self.reset_all_locations()


    def return_positions(self):
        # Return list of all marker locations.
        return self.old_locations



def main():

    cd = CameraDriver(list_of_markers_to_find, default_kernel_size=55, scaling_parameter=1000, downscale_factor=1)  # Best in robolab.
    t0 = time()

    # Calibration of setup in robolab, so that the coordinates correspond to real world coordinates.
    reference_point_locations_in_image = [[1328, 340], [874, 346], [856, 756], [1300, 762]]
    reference_point_locations_in_world_coordinates = [[0, 0], [300, 0], [300, 250], [0, 250]]
    perspective_corrector = PerspectiveCorrecter(reference_point_locations_in_image,
                                                 reference_point_locations_in_world_coordinates)

    while cd.running and stop_flag is False:
        (t1, t0) = (t0, time())
        if print_debug_messages is True:
            print("time for one iteration: %f" % (t0 - t1))
        cd.get_image()
        cd.process_frame()
        cd.draw_detected_markers()
        cd.show_processed_frame()
        cd.handle_keyboard_events()
        y = cd.return_positions()
        if publish_to_ros:
            ros_publisher.publish_marker_locations(y)
        else:
            for k in range(len(y)):
                try:
                    #pose_corrected = perspective_corrector.convertPose(y[k])
                    pose_corrected = y[k]
                    print("%8.3f %8.3f %8.3f %8.3f %s" % (pose_corrected.x,
                                                          pose_corrected.y,
                                                          pose_corrected.theta,
                                                          pose_corrected.quality,
                                                          pose_corrected.order))
                except Exception as e:
                    print("%s" % e)
    print("Stopping")


main()
