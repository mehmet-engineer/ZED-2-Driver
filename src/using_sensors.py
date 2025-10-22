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

    HZ = 1
    while zed_cam.is_opened():

        print("\n ------------------------------\n")
        
        zed_cam.update_sensor_data()

        trans = zed_cam.get_camera_translation()
        print("Camera Translation: ", trans, "meters")

        rotation = zed_cam.get_camera_rotation()
        print("Camera Rotation: \n", rotation)

        rpy = zed_cam.get_camera_rpy()
        print("Camera RPY: ", rpy, "degrees")

        quaternion = zed_cam.get_camera_quaternion()
        print("Camera Quaternion: ", quaternion)

        ang_vel = zed_cam.get_angular_velocity()
        print("Angular Velocity: ", ang_vel)

        lin_acc = zed_cam.get_linear_acceleration()
        print("Linear Acceleration: ", lin_acc)

        pressure = zed_cam.get_camera_pressure()
        print("Atmospheric Pressure: ", pressure, "hPa")

        altitude = zed_cam.get_camera_altitude()
        print("Atmospheric Altitude: ", altitude, "meters")

        magnetic_field = zed_cam.get_camera_magnetic_field()
        print("Magnetic Field: ", magnetic_field, "uT")

        temperature = zed_cam.get_temperature_left()
        print("Temperature Left: ", temperature, "C")

        zed_cam.rate_sleep(HZ)
        

    zed_cam.close()
    print("\n Camera closed.")


if __name__ == "__main__":
    main()