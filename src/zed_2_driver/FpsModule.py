import time
import cv2
import numpy as np


class FpsModule:

    def __init__(self, display_freq_ms: int):
        """
        Fps module for video stream.
        """

        self.fps = 0
        self.displayed_fps = 0
        self.prev_fps_time = 0.0
        self.text = ""
        self.middle_fps_level = 10.0
        self.good_fps_level = 30.0
        self.display_point = (5, 30)
        
        self.color_map = {
            "green": (0, 255, 0),
            "yellow": (0, 255, 255),
            "red": (0, 0, 255),
            "blue": (255, 0, 0),
            "orange": (0, 100, 255),
            "white": (255, 255, 255),
            "black": (0, 0, 0)
        }
        
        self.prev_display_time = 0.0
        self.display_freq_ms = display_freq_ms
        self.display_freq = display_freq_ms / 1000

    def calculate_fps(self) -> int:
        """
        Calculates (FPS) frames per second.
        Returns:
            int: FPS value.
        """

        current_time = time.time()

        try:
            self.fps = round(1 / (current_time - self.prev_fps_time))
        except ZeroDivisionError:
            pass
        
        self.prev_fps_time = current_time
        
        return self.fps
    
    def display_fps(self, img: np.ndarray):
        """
        Displays the FPS value on the image corner.
        Args:
            img (np.array): Image to display FPS.
        """
        
        current_time = time.time()

        if current_time - self.prev_display_time > self.display_freq:

            if self.fps >= self.good_fps_level:
                self.displayed_color = self.color_map["green"]

            elif self.fps >= self.middle_fps_level:
                self.displayed_color = self.color_map["orange"]

            else:
                self.displayed_color = self.color_map["red"]
            
            self.displayed_fps = self.fps
            self.text = "FPS:" + str(self.displayed_fps)
            self.prev_display_time = current_time
        
        cv2.putText(img, self.text, self.display_point, cv2.FONT_HERSHEY_SIMPLEX, 0.8, self.displayed_color, 2)
