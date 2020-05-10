import sys
import os
import pyzed.sl as sl
import cv2


def main():

    path = ["./svo/down/", "./svo/up/", "./svo/obstacle/", "./svo/flatten/", 
            "./svo/test_down/", "./svo/test_up/", "./svo/test_obstacle/", "./svo/test_flatten/"]
    # path = ["./svo/obstacle/", "./svo/flatten/"]
    for filepath in path:
        start_read_svo(filepath)

def start_read_svo(filepath):

    for file in os.listdir(filepath):
        if not file.endswith(".svo"):
            continue
        file = filepath + file
        print(file)
        print(file.rsplit(".", 1)[0])

        input_type = sl.InputType()
        input_type.set_from_svo_file(file)
        init = sl.InitParameters(input_t=input_type, svo_real_time_mode=False)
        init.depth_mode = sl.DEPTH_MODE.ULTRA
        cam = sl.Camera()
        status = cam.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            exit()

        runtime = sl.RuntimeParameters()
        runtime.sensing_mode = sl.SENSING_MODE.FILL
        # runtime.sensing_mode = sl.SENSING_MODE.STANDARD
        mat = sl.Mat()

        key = ''
        i = 0
        print("  Save the current image:     s")
        print("  Quit the video reading:     q\n")
        while key != 113:  # for 'q' key
            err = cam.grab(runtime)
            if err == sl.ERROR_CODE.SUCCESS:
                cam.retrieve_image(mat)
                rgb = mat.get_data()
                # cv2.imshow("RGB", rgb)
                cam.retrieve_image(mat, sl.VIEW.DEPTH)
                depth = mat.get_data()
                # cv2.imshow("Dep", depth)
                key = cv2.waitKey(1)
                if i>cam.get_svo_number_of_frames()*0.1\
                 and i<cam.get_svo_number_of_frames()*0.9:
                    cv2.imwrite(file.rsplit(".", 1)[0] + "_rgb_" + str(i) + ".png", rgb)
                    cv2.imwrite(file.rsplit(".", 1)[0] + "_depth_" + str(i) + ".jpg", depth)
                i += 1
            else:
                break
        cv2.destroyAllWindows()

        print_camera_information(cam)
        cam.close()
        print("\nFINISH")

def print_camera_information(cam):
    print()
    print("Distorsion factor of the right cam before calibration: {0}.".format(
        cam.get_camera_information().calibration_parameters_raw.right_cam.disto))
    print("Distorsion factor of the right cam after calibration: {0}.\n".format(
        cam.get_camera_information().calibration_parameters.right_cam.disto))

    print("Confidence threshold: {0}".format(cam.get_runtime_parameters().confidence_threshold))
    print("Depth min and max range values: {0}, {1}".format(cam.get_init_parameters().depth_minimum_distance,
                                                            cam.get_init_parameters().depth_maximum_distance))
    print("Resolution: {0}, {1}.".format(round(cam.get_camera_information().camera_resolution.width, 2), cam.get_camera_information().camera_resolution.height))
    print("Camera FPS: {0}".format(cam.get_camera_information().camera_fps))
    print("Frame count: {0}.\n".format(cam.get_svo_number_of_frames()))


if __name__ == "__main__":
    main()
