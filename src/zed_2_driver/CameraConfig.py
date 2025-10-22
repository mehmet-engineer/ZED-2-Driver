import pyzed.sl as sl

class CameraConfig():
    def __init__(self, fps: int, resolution: str, img_enhance: bool, depth_mode: str, min_dist: float, max_dist: float, log: bool):

        # class variables
        self.fps = fps
        self.resolution = resolution
        self.img_enhance = img_enhance
        self.depth_mode = depth_mode
        self.min_dist = min_dist
        self.max_dist = max_dist

        # parameter objects
        self.init_params = sl.InitParameters()
        self.runtime_params = sl.RuntimeParameters()

        # Camera and Resolution Settings
        self.resolutions = {
            'VGA': sl.RESOLUTION.VGA,
            'HD720': sl.RESOLUTION.HD720,
            'HD1080': sl.RESOLUTION.HD1080,
            'HD2K': sl.RESOLUTION.HD2K
        }
        
        if not resolution in self.resolutions.keys():
            err_msg = "Unsupported resolution! Supported resolutions are: " + ", ".join(self.resolutions.keys())
            raise ValueError(err_msg)
        
        self.init_params.camera_resolution = self.resolutions[resolution]
        self.init_params.camera_fps = fps
        self.init_params.svo_real_time_mode = False
        self.init_params.enable_image_enhancement = img_enhance

        # Coordinate System and Unit Settings
        self.init_params.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
        self.init_params.coordinate_units = sl.UNIT.METER

        # Depth Settings
        self.depth_modes = {
            'NONE': sl.DEPTH_MODE.NONE,
            'LOW': sl.DEPTH_MODE.PERFORMANCE,
            'MEDIUM': sl.DEPTH_MODE.QUALITY,
            'HIGH': sl.DEPTH_MODE.ULTRA
        }

        if not depth_mode in self.depth_modes.keys():
            err_msg = "Unsupported depth sensing mode! Supported modes are: " + ", ".join(self.depth_modes.keys())
            raise ValueError(err_msg)
        
        self.init_params.depth_mode = self.depth_modes[depth_mode]

        if depth_mode == "NONE":
            self.runtime_params.enable_depth = False
            self.runtime_params.enable_fill_mode = False
        if depth_mode == "LOW":
            self.runtime_params.enable_depth = True
            self.runtime_params.enable_fill_mode = False
        if depth_mode == "MEDIUM":
            self.runtime_params.enable_depth = True
            self.runtime_params.enable_fill_mode = False
        if depth_mode == "HIGH":
            self.runtime_params.enable_depth = True
            self.runtime_params.enable_fill_mode = True

        self.init_params.depth_stabilization = 1

        if depth_mode != "NONE":
            if min_dist < 0.3:
                err_msg = "Minimum distance must be at least 0.3 meters (30 cm) !!"
                raise ValueError(err_msg)
            if min_dist > 3.0:
                err_msg = "Minimum distance must be lower than 3.0 meters (300 cm) !!"
                raise ValueError(err_msg)
            
            self.init_params.depth_minimum_distance = min_dist
            self.init_params.depth_maximum_distance = max_dist

        self.runtime_params.confidence_threshold = 95

        # Sensor Settings
        self.init_params.sensors_required = True
        
        # Log Settings
        if log == True:
            self.init_params.sdk_verbose = 1
        else:
            self.init_params.sdk_verbose = 0
        

    def get_config(self) -> sl.InitParameters:
        return self.init_params
    
    def get_runtime_params(self) -> sl.RuntimeParameters:
        return self.runtime_params