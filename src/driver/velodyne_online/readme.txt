roslaunch velodyne_pointcloud VLP16_points.launch calibration:=/home/zqt/Desktop/velodyne_online/VLP-16.yaml 后面的参数可以不加,已经自己读进去了


rosbag record -O out /velodyne_points


