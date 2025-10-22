import numpy as np
import open3d as o3d

class Visualizer:

    def __init__(self):
        self.visualizer = o3d.visualization.Visualizer()
        self.mesh = None
        self.mesh_center = None
        self.prev_rot = np.eye(3)
    
        self.axis_align = np.array([
            [-1,  0,  0],
            [ 0, -1,  0],
            [ 0,  0,  1] 
        ])

    def rpy_to_rot_matrix(self, rotation_rpy: np.array, input_unit: str) -> np.ndarray:

        if input_unit == 'deg':
            rotation_rpy = np.radians(rotation_rpy)

        rx, ry, rz = rotation_rpy
        
        rot_x = np.array([
            [1, 0, 0],
            [0, np.cos(rx), -np.sin(rx)],
            [0, np.sin(rx), np.cos(rx)]
        ])
        rot_y = np.array([
            [np.cos(ry), 0, np.sin(ry)],
            [0, 1, 0],
            [-np.sin(ry), 0, np.cos(ry)]
        ])
        rot_z = np.array([
            [np.cos(rz), -np.sin(rz), 0],
            [np.sin(rz), np.cos(rz), 0],
            [0, 0, 1]
        ])
        
        rotation = rot_z @ rot_y @ rot_x
        
        return rotation

    def read_mesh(self, file_path: str, color: list):
        self.mesh = o3d.io.read_triangle_mesh(file_path)

        if self.mesh.is_empty():
            raise ValueError("Mesh is empty or file not found.")
        
        self.mesh.compute_vertex_normals()
        self.mesh.paint_uniform_color(color)
        self.mesh_center = self.mesh.get_center()

        fix_rot_rpy = [90, 0, 180]
        fix_rotation = self.rpy_to_rot_matrix(fix_rot_rpy, input_unit='deg')
        self.mesh.rotate(fix_rotation, center=self.mesh_center)

        self.visualizer.add_geometry(self.mesh)

    def update_mesh(self, new_rotation: np.ndarray):
        if self.mesh_center is None:
            raise ValueError("Mesh center is not set. Load a mesh first.")
        
        self.mesh.rotate(self.prev_rot.T, center=self.mesh_center)
        self.mesh.rotate(new_rotation, center=self.mesh_center)

        self.prev_rot = new_rotation.copy()

        self.visualizer.update_geometry(self.mesh)
        self.visualizer.poll_events()
        self.visualizer.update_renderer()

    def update_mesh_aligned(self, zed_rotation: np.ndarray):
        aligned_rotation = self.axis_align @ zed_rotation @ self.axis_align.T
        self.update_mesh(aligned_rotation)
    
    def visualize(self, window_name: str, width: int, height: int):
        self.visualizer.create_window(window_name=window_name, width=width, height=height)
        self.view_control = self.visualizer.get_view_control()

    def stop(self):
        self.visualizer.destroy_window()
        