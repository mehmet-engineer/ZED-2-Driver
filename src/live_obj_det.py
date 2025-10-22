import cv2
from zed_2_driver.FpsModule import FpsModule
from zed_2_driver.Zed2Camera import Zed2Camera
from zed_2_driver.CameraConfig import CameraConfig

def main():
    FPS = 30
    RESOLUTION = "HD720"
    IMG_ENHANCE = True
    DEPTH_MODE = "MEDIUM"
    MIN_DIST = 0.3
    MAX_DIST = 4.0
    LOG = False
    cam_config = CameraConfig(FPS, RESOLUTION, IMG_ENHANCE, DEPTH_MODE, MIN_DIST, MAX_DIST, LOG)

    display_fps = 300
    fps_module = FpsModule(display_fps)

    zed_cam = Zed2Camera(cam_config)
    success = zed_cam.connect()

    if not success:
        print("Could not connect to Camera!")
        return
    
    print("Successfully connected to Zed 2 Camera.")
    zed_cam.print_camera_info()
    print()

    print("Object detection module initializing ...")
    
    DIM = 2
    TRACK = False
    SEG = False
    DET_CONF = 90
    MAX_DET_RANGE = MAX_DIST
    init_success = zed_cam.init_object_detection(DIM, TRACK, SEG, DET_CONF, MAX_DET_RANGE)

    if init_success == False:
        raise ValueError("Object Detection Module could not be initialized !!")

    print("Live Capture is starting...")

    try: 
        while zed_cam.is_opened():
            success = zed_cam.grab_frame()

            if not success:
                print("Failed to capture image!")
                continue
            
            success, img = zed_cam.capture_left()

            success, objects = zed_cam.detect_objects()
            if not success:
                print("Failed to detect objects!")
                continue

            if objects.is_new:
                for obj in objects.object_list:
                    print()
                    print("id:", obj.id)
                    print("confidence:", obj.confidence)
                    print("position:", obj.position)
                    print("velocity:", obj.velocity)
                    print("bounding_box_3d:", obj.bounding_box)
                    print("bounding_box_2d:", obj.bounding_box_2d)
                    print("dimensions:", obj.dimensions)
            
            fps_module.calculate_fps()
            fps_module.display_fps(img)
            cv2.imshow("Live Camera", img)

            cv2.waitKey(1)

    except KeyboardInterrupt:
        print("Live Capture interrupted by user.")

    cv2.destroyAllWindows()
    zed_cam.close()
    print("Live Capture ended.")


if __name__ == "__main__":
    main()