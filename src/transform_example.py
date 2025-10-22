from zed_2_driver.Transform import Transform

def main():

    print("Transform Module working ...")
    transformer = Transform()
 
    # Camera pose with respect to world frame
    cam_euler_deg = [0, 0, -135]
    cam_translation = [0.18, 0.15, -0.2]
    transformer.move_camera(cam_euler_deg, cam_translation)

    # Detected object pose with respect to camera frame
    euler_deg = [0, 0, 0]
    translation = [0.05, 0.1, 0.3]
    world_target = transformer.calculate_world_target(euler_deg, translation)
    position = world_target.get_position()
    quaternion = world_target.get_quaternion()
    euler_deg = world_target.get_euler_deg()
    
    # print results
    print("Camera Transform --> Euler:", cam_euler_deg, "Translation:", cam_translation)
    print("Detected Object from camera --> Euler:", euler_deg, "Translation:", translation)
    print()
    print("World Target --> Position:", position)
    print("World Target --> Quaternion:", quaternion)
    print("World Target --> Euler:", euler_deg)

if __name__ == "__main__":
    main()