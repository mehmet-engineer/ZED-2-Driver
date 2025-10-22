from zed_2_driver.Zed2Camera import Zed2Camera
from zed_2_driver.CameraConfig import CameraConfig
from zed_2_driver.PointCloud import PointCloud

def main():
    FPS = 30
    RESOLUTION = "HD2K"
    IMG_ENHANCE = False
    DEPTH_MODE = "HIGH"
    MIN_DIST = 0.3
    MAX_DIST = 1.5
    LOG = False
    cam_config = CameraConfig(FPS, RESOLUTION, IMG_ENHANCE, DEPTH_MODE, MIN_DIST, MAX_DIST, LOG)

    pc_processor = PointCloud()

    zed_cam = Zed2Camera(cam_config)
    success = zed_cam.connect()

    if not success:
        print("Could not connect to Camera!")
        return
    
    print("Successfully connected to Zed 2 Camera.")
    zed_cam.print_camera_info()
    print()

    print("Point cloud is generating...")
    success = zed_cam.grab_frame()

    if not success:
        print("Failed to capture depth image for point cloud generation!")
        return
    
    PATH = "assets/point_cloud.pcd"
    success = zed_cam.save_point_cloud(PATH)

    x = 640
    y = 360
    pc_data = zed_cam.get_point_cloud_value(x, y)
    print("Point Cloud Data: ", pc_data)

    if not success:
        print("Failed to generate point cloud!")

    pc_processor.load_point_cloud_file(PATH)
    pc_processor.visualize_point_cloud("Zed 2 Point Cloud")
    
    zed_cam.close()
    print("\n Camera closed.")


if __name__ == "__main__":
    main()