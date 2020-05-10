########################################################################
#
# Copyright (c) 2020, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

import pyzed.sl as sl
import numpy as np
import cv2
import math

def main():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080  # Use HD1080 video mode
    # https://www.stereolabs.com/docs/depth-sensing/depth-settings/
    # Several depth modes are available to improve performance in certain applications
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA
    init_params.coordinate_units = sl.UNIT.METER
    init_params.camera_fps = 30
    # init_params.depth_minimum_distance = 0.15
    init_params.depth_maximum_distance = 20
    # zed.set_depth_max_range_value(20)

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        exit(1)
    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()

    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.STANDARD
    runtime_parameters.sensing_mode = sl.SENSING_MODE.FILL  # FILL STANDARD
    # Setting the depth confidence parameters
    # https://www.stereolabs.com/docs/ros/zed-node/
    # A lower value means more confidence and precision
    # [1, 100], default: 100
    runtime_parameters.confidence_threshold = 100
    runtime_parameters.textureness_confidence_threshold = 100

    while True:
        # Grab an image, a RuntimeParameters object must be given to grab()
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            
            zed.retrieve_image(image, sl.VIEW.LEFT)
            ### The 32-bit depth map can be displayed as a grayscale 8-bit image.
            ### 0 (black) represents the most distant possible depth value. We call this process depth normalization
            ### 不太準，可能用點雲自己轉換比較準?
            zed.retrieve_image(depth, sl.VIEW.DEPTH)
            ### extract the depth map of a scene
            # zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
            
            ############
            img_process(image, depth)
            dis = get_depth_value(depth)
            # print("D: ", dis)
            dis = get_point_cloud_depth_value(point_cloud)
            print("p: ", dis)
            ############
    # Close the camera
    zed.close()

def img_process(image_mat, depth_mat):
    img = image_mat.get_data()
    depth_img = depth_mat.get_data()
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (840, 640))
    depth_img = cv2.resize(depth_img, (840, 640))
    cv2.imshow('Frame', np.hstack([img]))
    cv2.imshow('Frame2', np.hstack([depth_img]))
    # cv2.imshow('Frame', [img, depth_img])
    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        sys.exit(0)

def get_depth_value(mat, x=None, y=None):
    '''
    By default, depth values are expressed in millimeters
    '''
    if x == None:
        x = round(mat.get_width() / 2)
        y = round(mat.get_height() / 2)
    return mat.get_value(x,y)
def get_point_cloud_depth_value(mat, x=None, y=None):
    '''
    By default, depth values are expressed in millimeters
    '''
    if x == None:
        x = round(mat.get_width() / 2)
        y = round(mat.get_height() / 2)

    err, point_cloud_value = mat.get_value(x, y)
    distance = math.sqrt(point_cloud_value[0] * point_cloud_value[0] +
                                 point_cloud_value[1] * point_cloud_value[1] +
                                 point_cloud_value[2] * point_cloud_value[2])
    return distance

if __name__ == "__main__":
    main()
