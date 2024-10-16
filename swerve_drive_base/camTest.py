import pyzed.sl as sl
import time

def main():
    # Create a Camera object
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.AUTO # Use HD720 or HD1200 video mode (default fps: 60)
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP # Use a right-handed Y-up coordinate system
    init_params.coordinate_units = sl.UNIT.METER  # Set units in meters

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Camera Open : "+repr(err)+". Exit program.")
        exit()


    # Enable positional tracking with default parameters
    py_transform = sl.Transform()  # First create a Transform object for TrackingParameters object
    tracking_parameters = sl.PositionalTrackingParameters(_init_pos=py_transform)
    err = zed.enable_positional_tracking(tracking_parameters)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Enable positional tracking : "+repr(err)+". Exit program.")
        zed.close()
        exit()

    # Track the camera position during 1000 frames
    i = 0
    zed_pose = sl.Pose()

    zed_sensors = sl.SensorsData()
    runtime_parameters = sl.RuntimeParameters()
    
    can_compute_imu = zed.get_camera_information().camera_model != sl.MODEL.ZED
    while i < 1000:
        time.sleep(1)
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Get the pose of the left eye of the camera with reference to the world frame
            zed.get_position(zed_pose, sl.REFERENCE_FRAME.WORLD)
            

            # Display the translation and timestamp
            py_translation = sl.Translation()
            tx = round(zed_pose.get_translation(py_translation).get()[0], 3)
            ty = round(zed_pose.get_translation(py_translation).get()[1], 3)
            tz = round(zed_pose.get_translation(py_translation).get()[2], 3)
            print("Translation: Tx: {0}, Ty: {1}, Tz {2}, Timestamp: {3}\n".format(tx, ty, tz, zed_pose.timestamp.get_milliseconds()))

            # Display the orientation quaternion
            py_orientation = sl.Orientation()
            ox = round(zed_pose.get_orientation(py_orientation).get()[0], 3)
            oy = round(zed_pose.get_orientation(py_orientation).get()[1], 3)
            oz = round(zed_pose.get_orientation(py_orientation).get()[2], 3)
            ow = round(zed_pose.get_orientation(py_orientation).get()[3], 3)
            print("Orientation: Ox: {0}, Oy: {1}, Oz {2}, Ow: {3}\n".format(ox, oy, oz, ow))
            
            if can_compute_imu:
                zed.get_sensors_data(zed_sensors, sl.TIME_REFERENCE.IMAGE)
                zed_imu = zed_sensors.get_imu_data()
                
                

                # Display the IMU orientation quaternion
                zed_imu_pose = sl.Transform()
                ox = round(zed_imu.get_pose(zed_imu_pose).get_orientation().get()[0], 3)
                oy = round(zed_imu.get_pose(zed_imu_pose).get_orientation().get()[1], 3)
                oz = round(zed_imu.get_pose(zed_imu_pose).get_orientation().get()[2], 3)
                ow = round(zed_imu.get_pose(zed_imu_pose).get_orientation().get()[3], 3)
                print("IMU Orientation: Ox: {0}, Oy: {1}, Oz {2}, Ow: {3}\n".format(ox, oy, oz, ow))

            i = i + 1
    # Close the camera
    zed.close()

if __name__ == "__main__":
    main()