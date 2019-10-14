
<launch>

    <include file="$(find velodyne_pointcloud)/launch/VLP16_points.launch"/>
    <include file="$(find realsense2_camera)/launch/rs_camera.launch"/>

    <include file="$(find pointcloud_to_laserscan)/launch/pointcloud_to_laserscan.launch"/>
    <include file="$(find ray_ground_filter)/launch/ray_ground_filter.launch" />
    <include file="$(find elevation_mapping_demos)/launch/realsense_demo.launch" />


    <include file="$(find ndt_localization)launch/ndt_localization.launch"/>

    <!--发布静态TF变换-->
    <node pkg="tf" type="static_transform_publisher" name="base2laser" args="0.2 0 0.8 0 0 0 1 /base_link /velodyne 100" />
    <node pkg="tf" type="static_transform_publisher" name="base2camera" args="0.25 0 0.7 0 0.38 0 1 /base_link /camera_link 100" />

    
    <!--RVIZ-->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find elevation_mapping_demos)/rviz/elevation_map_visualization1.rviz" />
</launch>


