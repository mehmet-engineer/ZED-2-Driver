import open3d

class PointCloud:

    def __init__(self):
        self.point_cloud = open3d.geometry.PointCloud()

    def load_point_cloud_file(self, file_path: str):
        self.point_cloud = open3d.io.read_point_cloud(file_path)
        if len(self.point_cloud.points) < 1:
            raise ValueError("Point cloud is empty or file not found.")
    
    def visualize_point_cloud(self, window_name: str):
        if len(self.point_cloud.points) < 1:
            raise ValueError("Point cloud is empty. Load a point cloud file first.")
        
        open3d.visualization.draw_geometries([self.point_cloud], window_name=window_name)

    def save_point_cloud_file(self, file_path: str):
        if len(self.point_cloud.points) < 1:
            raise ValueError("Point cloud is empty. Load a point cloud file first.")
        
        open3d.io.write_point_cloud(file_path, self.point_cloud)
