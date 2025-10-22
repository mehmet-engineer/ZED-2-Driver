import numpy as np
from scipy.spatial import transform

class CameraTransform:

    def __init__(self, t_matrix=None, quat=None, euler_deg=None, rotation=None, translation=None):

        if t_matrix is not None:
            self.T_matrix = t_matrix
            self.rotation_matrix = self._get_rotation_from_transform(self.T_matrix)
            self.translation = self._get_translation_from_transform(self.T_matrix)
            self.quaternion = self._rotation_to_quaternion(self.rotation_matrix)
            self.euler_deg = self._rotation_to_euler(self.rotation_matrix)

        elif (quat is not None) and (translation is not None):
            self.quaternion = np.array(quat)
            self.rotation_matrix = self._quaternion_rotation_matrix()
            self.translation = np.array(translation)
            self.T_matrix = self._get_transform_matrix(self.rotation_matrix, self.translation)
            self.euler_deg = self._rotation_to_euler(self.rotation_matrix)
        
        elif (euler_deg is not None) and (translation is not None):
            self.euler_deg = np.array(euler_deg)
            self.translation = np.array(translation)
            self.rotation_matrix = self._euler_to_rotation_matrix(euler_deg, input_unit="deg")
            self.T_matrix = self._get_transform_matrix(self.rotation_matrix, self.translation)
            self.quaternion = self._euler_to_quaternion(euler_deg, input_unit="deg")

        elif (rotation is not None) and (translation is not None):
            self.rotation_matrix = rotation
            self.translation = np.array(translation)
            self.T_matrix = self._get_transform_matrix(self.rotation_matrix, self.translation)
            self.quaternion = self._rotation_to_quaternion(self.rotation_matrix)
            self.euler_deg = self._rotation_to_euler(self.rotation_matrix)

        else:
            raise ValueError("Invalid parameter initialization !!")
    
    def _get_rotation_from_transform(self, t_matrix: np.ndarray) -> np.ndarray:
        print(t_matrix)
        rotation = t_matrix[:3, :3]
        return rotation
    
    def _get_translation_from_transform(self, t_matrix: np.ndarray) -> np.ndarray:
        translation = t_matrix[:3, 3]
        return translation

    def _get_transform_matrix(self, rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = rotation
        T[:3, 3] = translation

        return T
    
    def _euler_to_rotation_matrix(self, euler, input_unit: str) -> np.ndarray:
        if input_unit == "deg":
            euler = np.radians(euler)

        roll, pitch, yaw = euler

        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll),  np.cos(roll)]
        ])
        Ry = np.array([
            [ np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)]
        ])
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [0, 0, 1]
        ])

        R = Rz @ Ry @ Rx

        return R
    
    def _rotation_to_euler(self, rotation: np.ndarray) -> np.ndarray:
        rot = transform.Rotation.from_matrix(rotation)
        euler_deg = rot.as_euler('xyz', degrees=True)
        return euler_deg
    
    def _euler_to_quaternion(self, euler: np.ndarray, input_unit: str) -> np.ndarray:
        if input_unit == "deg":
            euler = np.radians(euler)
        
        rot = transform.Rotation.from_euler('xyz', euler, degrees=False)
        quaternion = rot.as_quat()

        return quaternion
    
    def _rotation_to_quaternion(self, rotation: np.ndarray) -> np.ndarray:
        rot = transform.Rotation.from_matrix(rotation)
        quaternion = rot.as_quat()
        return quaternion
        
    def _quaternion_rotation_matrix(self, quaternion: np.ndarray) -> np.ndarray:
        rot = transform.Rotation.from_quat(quaternion)
        rot_matrix = rot.as_matrix()               
        return rot_matrix

    def get_position(self) -> np.ndarray:
        return self.translation
    
    def get_rotation(self) -> np.ndarray:
        return self.rotation_matrix
    
    def get_quaternion(self) -> np.ndarray:
        return self.quaternion
    
    def get_euler_deg(self) -> np.ndarray:
        return self.euler_deg
    
    def get_transformation(self) -> np.ndarray:
        return self.T_matrix
    

class Transform:

    def __init__(self):
        self.camera_transform = None
    
    def move_camera(self, euler_deg: list[float], translation: list[float]):
        self.camera_transform = CameraTransform(euler_deg=euler_deg, translation=translation)
    
    def calculate_world_target(self, euler_deg: list[float], translation: list[float]) -> CameraTransform:
        target_transform = CameraTransform(euler_deg=euler_deg, translation=translation)
        world_target = self.camera_transform.get_transformation() @ target_transform.get_transformation()
        return CameraTransform(t_matrix=world_target)
    
    def get_camera_transform(self) -> CameraTransform:
        return self.camera_transform