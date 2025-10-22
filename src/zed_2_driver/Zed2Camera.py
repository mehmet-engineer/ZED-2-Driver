import cv2
import time
import enum
import numpy as np
import pyzed.sl as sl
from zed_2_driver.CameraConfig import CameraConfig

class Zed2Camera:
    def __init__(self, cam_config: CameraConfig):
        self.zed_cam = sl.Camera()
        self.cam_config = cam_config
        self.init_params = self.cam_config.get_config()
        self.runtime_params = self.cam_config.get_runtime_params()
        self.sensor_data = sl.SensorsData()
        self.is_initialized = False
        self.is_obj_det_enabled = False
        self.last_frame_timestamp = None

    # CONNECTION /////////////////////////////////////////////////////////////////

    def connect(self) -> bool:
        if not self.zed_cam.open(self.init_params) == sl.ERROR_CODE.SUCCESS:
            self.is_initialized = False
        else:
            self.is_initialized = True
        
        return self.is_initialized
    
    def is_opened(self) -> bool:
        state = self.zed_cam.is_opened()
        return state
    
    def print_camera_info(self):
        print("Serial No: " + str(self.get_serial_number()))
        print("Camera Model: " + str(self.get_camera_model_name()))
        print("Camera Resolution: " + str(self.get_camera_resolution()))
        print("Camera FPS: " + str(self.get_camera_fps()))
        print("Firmware Version: " + str(self.get_firmware_version()))
        print("SDK Version: " + str(self.get_sdk_version()))
        
    def get_camera_info_str(self) -> str:
        string = (
            f"Serial No: {self.get_serial_number()}\n"
            f"Camera Model: {self.get_camera_model_name()}\n"
            f"Camera Resolution: {self.get_camera_resolution()}\n"
            f"Camera FPS: {self.get_camera_fps()}\n"
            f"Firmware Version: {self.get_firmware_version()}\n"
            f"SDK Version: {self.get_sdk_version()}"
        )
        return string.strip()
    
    # OBJECT DETECTION MODULE ////////////////////////////////////////////////////
    
    def init_object_detection(self, dim: int, track: bool, seg: bool, det_conf: float, max_det_range: float) -> bool:
        self.is_obj_det_enabled = True
        self.detect_params = sl.ObjectDetectionParameters()
        self.detect_runtime_params = sl.ObjectDetectionRuntimeParameters()

        self.detect_dim = dim
        self.detect_track = track
        self.detect_seg = seg
        self.detect_conf = det_conf
        self.detect_max_range = max_det_range

        # detection model
        # TO DO  model selection ?????
        self.detect_params.detection_model = sl.OBJECT_DETECTION_MODEL.MULTI_CLASS_BOX_FAST
        self.detect_params.filtering_mode = sl.OBJECT_FILTERING_MODE.NMS3D

        # detection range and confidence
        self.detect_runtime_params.detection_confidence_threshold = det_conf
        self.detect_params.max_range = max_det_range

        # tracking
        self.detect_params.enable_tracking = track
        if track == True:
            self.zed_cam.enable_positional_tracking()
        
        # segmentation
        if (dim == 2) and (seg == True):
            self.detect_params.enable_segmentation = True
        else:
            self.detect_params.enable_segmentation = False

        # final success
        err = self.zed_cam.enable_object_detection(self.detect_params)
        if err == sl.ERROR_CODE.SUCCESS:
            self.zed_cam.set_object_detection_runtime_parameters(self.runtime_params)
            return True
        else:
            return False
        
    def detect_objects(self) -> tuple:
        objects = sl.Objects()
        success = self.zed_cam.retrieve_objects(objects)
    
        if success == sl.ERROR_CODE.SUCCESS:
            return (True, objects)
        else:
            return (False, objects)
    
    # CAMERA INTERACTION /////////////////////////////////////////////////////////
    
    def grab_frame(self) -> bool:
        if not self.is_initialized:
            raise RuntimeError("Camera is not initialized.")
        
        grab_success = (self.zed_cam.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS)
        if grab_success:
            self.last_frame_timestamp = self.zed_cam.get_timestamp(sl.TIME_REFERENCE.IMAGE)
        
        return grab_success

    def capture_left(self) -> np.ndarray:
        if not self.is_initialized:
            raise RuntimeError("Camera is not initialized.")
        
        if self.last_frame_timestamp == None:
            raise RuntimeError("No frame has been grabbed yet. Call grab_frame() first.")
        
        image = sl.Mat()
        self.zed_cam.retrieve_image(image, sl.VIEW.LEFT)
        bgra = image.get_data()
        
        bgr_img = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        return bgr_img
        
    def capture_right(self) -> np.ndarray:
        if not self.is_initialized:
            raise RuntimeError("Camera is not initialized.")
        
        if self.last_frame_timestamp == None:
            raise RuntimeError("No frame has been grabbed yet. Call grab_frame() first.")
        
        image = sl.Mat()
        self.zed_cam.retrieve_image(image, sl.VIEW.RIGHT)
        bgra = image.get_data()
        bgr_img = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        return bgr_img
        
    def capture_stereo(self) -> np.ndarray:
        if not self.is_initialized:
            raise RuntimeError("Camera is not initialized.")
        
        if self.last_frame_timestamp == None:
            raise RuntimeError("No frame has been grabbed yet. Call grab_frame() first.")
                
        image = sl.Mat()
        self.zed_cam.retrieve_image(image, sl.VIEW.SIDE_BY_SIDE)
        bgra = image.get_data()
        bgr_img = cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
        return bgr_img
    
    def _convert_to_colormap(self, depth_img: np.ndarray) -> np.ndarray:
        min_val = 0
        max_val = 255
        depth_norm = cv2.normalize(depth_img, None, min_val, max_val, cv2.NORM_MINMAX)
        depth_norm = (max_val - depth_norm).astype('uint8')
        
        depth_colored = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

        return depth_colored
    
    def _get_depth_data(self, depth_direction: enum.Enum) -> sl.Mat:
        if not self.is_initialized:
            raise RuntimeError("Camera is not initialized.")
        
        if self.last_frame_timestamp == None:
            raise RuntimeError("No frame has been grabbed yet. Call grab_frame() first.")
        
        depth_data = sl.Mat()
        self.zed_cam.retrieve_measure(depth_data, depth_direction)
        return depth_data
        
    def capture_depth_left(self, colored: bool) -> np.ndarray:
        depth = self._get_depth_data(sl.MEASURE.DEPTH)
        depth_data = depth.get_data()
        depth_saturated = np.nan_to_num(depth_data, nan=0.0, posinf=0.0, neginf=0.0)

        if colored:
            depth_colored = self._convert_to_colormap(depth_saturated)
            return depth_colored
        else:
            return depth_saturated

    def capture_depth_right(self, colored: bool) -> tuple:
        depth = self._get_depth_data(sl.MEASURE.DEPTH_RIGHT)
        depth_data = depth.get_data()
        depth_saturated = np.nan_to_num(depth_data, nan=0.0, posinf=0.0, neginf=0.0)

        if colored:
            depth_colored = self._convert_to_colormap(depth_saturated)
            return depth_colored
        else:
            return depth_saturated
        
    def get_depth_from_left(self, x: int, y: int) -> float:
        depth_data = self._get_depth_data(sl.MEASURE.DEPTH)
        depth_value = depth_data.get_value(x, y)[1]
        return depth_value
    
    def get_depth_from_right(self, x: int, y: int) -> float:
        depth_data = self._get_depth_data(sl.MEASURE.DEPTH_RIGHT)
        depth_value = depth_data.get_value(x, y)[1]
        return depth_value
    
    def _get_point_cloud_data(self) -> sl.Mat:
        if not self.is_initialized:
            raise RuntimeError("Camera is not initialized.")
        
        if self.last_frame_timestamp == None:
            raise RuntimeError("No frame has been grabbed yet. Call grab_frame() first.")
        
        point_cloud = sl.Mat()
        self.zed_cam.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
        return point_cloud
    
    def filter_inf_point_cloud(self, point_cloud: np.array) -> np.array:
        point_cloud = np.array(point_cloud, dtype=np.float32)
        xyz = point_cloud[:, :, :3]
        invalid_mask = np.logical_not(np.isfinite(xyz))
        xyz[invalid_mask] = 0.0
        point_cloud[:, :, :3] = xyz
        return point_cloud

    def capture_point_cloud(self, filter_inf: bool) -> np.array:
        point_cloud_data = self._get_point_cloud_data()
        point_cloud = point_cloud_data.get_data()
        
        if filter_inf:
            point_cloud = self.filter_inf_point_cloud(point_cloud)

        return point_cloud
        
    def save_point_cloud(self, file_path: str) -> bool:
        point_cloud_data = self._get_point_cloud_data()
        try:
            point_cloud_data.write(file_path)
            return True
        except:
            return False
        
    def get_point_cloud_value(self, x: int, y: int) -> list[float]:
        point_cloud_data = self._get_point_cloud_data()
        pc_value = point_cloud_data.get_value(x, y)[1]
        
        px = float(pc_value[0])
        py = float(pc_value[1])
        pz = float(pc_value[2])
        color = float(pc_value[3])
        
        return [px, py, pz, color]

    # RUNTIME METHODS //////////////////////////////////////////////////////////////

    def sleep_for(self, seconds: float):
        time.sleep(seconds)

    def sleep_for_milliseconds(self, milliseconds: int):
        time.sleep(milliseconds / 1000.0)

    def rate_sleep(self, hz_rate: float):
        sleep_duration = 1.0 / hz_rate
        time.sleep(sleep_duration)

    def get_current_fps(self) -> float:
        if not self.is_initialized:
            raise RuntimeError("Camera is not initialized.")
        
        fps = self.zed_cam.get_current_fps()
        return fps
        
    # SENSOR METHODS //////////////////////////////////////////////////////////////

    def update_sensor_data(self):
        if not self.is_initialized:
            raise RuntimeError("Camera is not initialized.")
        
        self.zed_cam.get_sensors_data(self.sensor_data, sl.TIME_REFERENCE.CURRENT)
    
    def _get_imu_data(self) -> sl.IMUData:
        imu_data = self.sensor_data.get_imu_data()
        
        if imu_data.is_available:
            return imu_data
        else:
            raise RuntimeError("IMU data is not available.")

    def get_camera_translation(self):
        imu = self._get_imu_data()
        trans = imu.get_pose().get_translation().get()
        return trans
    
    def get_camera_rotation(self) -> np.ndarray:
        imu = self._get_imu_data()
        rot = imu.get_pose().get_rotation_matrix().r
        return rot
    
    def get_camera_rpy(self) -> list[float]:
        imu = self._get_imu_data()
        rpy = imu.get_pose().get_euler_angles(radian=False)
        return rpy
    
    def get_camera_quaternion(self) -> list[float]:
        imu = self._get_imu_data()
        quaternion = imu.get_pose().get_orientation().get()
        return quaternion
    
    def get_angular_velocity(self) -> list[float]:
        imu = self._get_imu_data()
        angular_velocity = imu.get_angular_velocity()
        return angular_velocity
    
    def get_linear_acceleration(self) -> list[float]:
        imu = self._get_imu_data()
        linear_acceleration = imu.get_linear_acceleration()
        return linear_acceleration
    
    def get_camera_pressure(self) -> float:
        pressure = self.sensor_data.get_barometer_data().pressure
        return pressure
    
    def get_camera_altitude(self) -> float:
        altitude = self.sensor_data.get_barometer_data().relative_altitude
        return altitude
        
    def get_camera_magnetic_field(self) -> list[float]:
        magnetic_field = self.sensor_data.get_magnetometer_data().get_magnetic_field_calibrated()
        return magnetic_field

    def get_temperature_left(self) -> float:
        temperature = self.sensor_data.get_temperature_data().get(sl.SENSOR_LOCATION.ONBOARD_LEFT)
        return temperature
    
    def get_temperature_right(self) -> float:
        temperature = self.sensor_data.get_temperature_data().get(sl.SENSOR_LOCATION.ONBOARD_RIGHT)
        return temperature
    
    def get_imu_temperature(self) -> float:
        temperature = self.sensor_data.get_temperature_data().get(sl.SENSOR_LOCATION.IMU)
        return temperature
            
    # CAMERA INFORMATION ///////////////////////////////////////////////////////////

    def get_serial_number(self) -> int:
        if not self.is_initialized:
            raise RuntimeError("Camera is not initialized.")
        ser_num = self.zed_cam.get_camera_information().serial_number
        return ser_num
    
    def get_camera_model_name(self) -> str:
        if not self.is_initialized:
            raise RuntimeError("Camera is not initialized.")
        cam_model = self.zed_cam.get_camera_information().camera_model.name
        return cam_model
    
    def get_camera_resolution(self) -> tuple:
        if not self.is_initialized:
            raise RuntimeError("Camera is not initialized.")
        cam_res = self.zed_cam.get_camera_information().camera_configuration.resolution
        return (cam_res.width, cam_res.height)
    
    def get_camera_fps(self) -> float:
        if not self.is_initialized:
            raise RuntimeError("Camera is not initialized.")
        cam_fps = self.zed_cam.get_camera_information().camera_configuration.fps
        return cam_fps
    
    def get_firmware_version(self) -> int:
        if not self.is_initialized:
            raise RuntimeError("Camera is not initialized.")
        firm = self.zed_cam.get_camera_information().camera_configuration.firmware_version
        return firm
    
    def get_sdk_version(self) -> str:
        if not self.is_initialized:
            raise RuntimeError("Camera is not initialized.")
        sdk = self.zed_cam.get_sdk_version()
        return sdk
    
    # DESTRUCTOR /////////////////////////////////////////////////////////////////

    def __del__(self):
        self.close()

    def close(self):
        if self.is_obj_det_enabled:
            self.zed_cam.disable_object_detection()
        self.zed_cam.close()
