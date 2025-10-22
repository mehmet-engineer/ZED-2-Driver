from zed_2_driver.Zed2Camera import Zed2Camera
from zed_2_driver.CameraConfig import CameraConfig
from zed_2_driver.Visualizer import Visualizer

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

    visualizer = Visualizer()
    window_name = "ZED2 Camera Rotation"
    width = 1280
    height = 720
    visualizer.visualize(window_name=window_name, width=width, height=height)

    color = [0.5, 0.5, 0.5]
    mesh_path = "assets/ZED2.stl"
    visualizer.read_mesh(mesh_path, color)

    HZ = 30
    while zed_cam.is_opened():
        
        zed_cam.update_sensor_data()

        # get camera rotation
        rotation = zed_cam.get_camera_rotation()
        print("Camera Rotation: \n", rotation)

        # update mesh respect to aligned axeses
        visualizer.update_mesh_aligned(rotation)

        zed_cam.rate_sleep(HZ)
        print()

    visualizer.stop()
    zed_cam.close()
    print("\n Camera closed.")


if __name__ == "__main__":
    main()