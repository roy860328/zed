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

import sys
import os
import pyzed.sl as sl
import cv2
from signal import signal, SIGINT

cam = sl.Camera()

def handler(signal_received, frame):
    cam.disable_recording()
    cam.close()
    sys.exit(0)

signal(SIGINT, handler)

def main():
    if not sys.argv or len(sys.argv) != 2:
        print("Only the path of the output SVO file should be passed as argument.")
        exit(1)

    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_resolution = sl.RESOLUTION.HD1080
    # init.depth_mode = sl.DEPTH_MODE.NONE

    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit(1)

    path_output = sys.argv[1] + "/" + get_last_file_index(sys.argv[1]) +".svo"
    # 檔案大小差距很大
    # recording_param = sl.RecordingParameters(path_output, sl.SVO_COMPRESSION_MODE.LOSSLESS)
    # 影像損失程度不高
    recording_param = sl.RecordingParameters(path_output, sl.SVO_COMPRESSION_MODE.H264)
    err = cam.enable_recording(recording_param)
    if err != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit(1)

    runtime = sl.RuntimeParameters()
    mat = sl.Mat()
    print("SVO is Recording, use Ctrl-C to stop.")
    frames_recorded = 0

    while True:
        if cam.grab(runtime) == sl.ERROR_CODE.SUCCESS :
            frames_recorded += 1
            cam.retrieve_image(mat)
            cv2.imshow("RGB", mat.get_data())
            cam.retrieve_image(mat, sl.VIEW.DEPTH)
            cv2.imshow("Dep", mat.get_data())
            key = cv2.waitKey(1)
            print("Frame count: " + str(frames_recorded), end="\r")

def get_last_file_index(path):
    last = "0"
    for f in os.listdir(path):
        if int(last) < int(f.split(".")[0]):
            last = f.split(".")[0]
    last = int(last)+1
    return str(last)

if __name__ == "__main__":
    main()