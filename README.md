## RUN ##
rosrun rqt_plot rqt_plot /odom/pose/pose/position/z /ual/odom/pe/pose/position/z
rosrun read_laser odom

## IMAGE ##
roslaunch realsense2_camera rs_camera.launch 
rqt_image_view /camera/color/camera_raw
rosrun ros_mono_vo mono_vo_node
roslaunch orb_slam2_ros orb_slam2_d435_mono.launch

## LIDAR ##
ls -l /dev |grep ttyUSB
sudo chmod 666 /dev/ttyUSB0
roslaunch rplidar_ros view_rplidar_s1.launch
rosbag record /scan

roslaunch read_laser visualice.launch

## BAGS ##
rosbag play -r 5 2021-03-11-11-51-03.bag --clock

rosbag record /camera/color/camera_info /camera/color/image_raw /camera/depth/image_rect_raw 
-O nombre.bag 
-a -> graba todos los topics

##########################################################
sudo chmod 777 /dev/my_lidar PERMISOS DE PUERTO
roslaunch rplidar_ros sensors.launch
rostopic echo /camera/color/image_raw
rostopic echo /scan
rostopic echo /mavros/imu/data_raw
roslaunch rplidar_ros record.launch
