from zed_2_driver.Zed2Camera import Zed2Camera
from zed_2_driver.CameraConfig import CameraConfig

def main():
    FPS = 30
    RESOLUTION = "HD720"
    IMG_ENHANCE = False
    DEPTH_MODE = "NONE"
    MIN_DIST = 0
    MAX_DIST = 0
    LOG = True
    cam_config = CameraConfig(FPS, RESOLUTION, IMG_ENHANCE, DEPTH_MODE, MIN_DIST, MAX_DIST, LOG)

    zed_cam = Zed2Camera(cam_config)
    success = zed_cam.connect()

    if success:
        print("Successfully connected to Zed 2 Camera.")
        zed_cam.print_camera_info()

    else:
        print("Could not connect!")
    
    zed_cam.close()
    print("\n Camera closed.")


if __name__ == "__main__":
    main()