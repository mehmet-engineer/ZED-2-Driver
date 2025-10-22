import cv2
from zed_2_driver.FpsModule import FpsModule
from zed_2_driver.Zed2Camera import Zed2Camera
from zed_2_driver.CameraConfig import CameraConfig

def main():
    FPS = 30
    RESOLUTION = "HD720"
    IMG_ENHANCE = False
    DEPTH_MODE = "HIGH"
    MIN_DIST = 0.3
    MAX_DIST = 5.0
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

    print("Live Capture is starting...")

    try: 
        while zed_cam.is_opened():
            success = zed_cam.grab_frame()
            
            if not success:
                print("Failed to capture depth image!")
                continue

            depth_colored = zed_cam.capture_depth_left(colored=True)

            x = 640
            y = 360
            cv2.circle(depth_colored, (x,y), 2, [0, 0, 0], 5)
            
            distance = zed_cam.get_depth_from_left(x, y)
            print("Distance in (" + str(x) + ", " + str(y) + ") --> " + str(distance) + " meter.")

            fps_module.calculate_fps()
            fps_module.display_fps(depth_colored)
            
            cv2.imshow("Live Colored Depth", depth_colored)

            cv2.waitKey(1)

    except KeyboardInterrupt:
        print("Live Capture interrupted by user.")

    cv2.destroyAllWindows()
    zed_cam.close()
    print("Live Capture ended.")


if __name__ == "__main__":
    main()