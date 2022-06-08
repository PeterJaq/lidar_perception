# lidar_perception

This is a project dedicated to solving the LIDAR perception in the field of autonomous driving. The project contains LIDAR detection, ground segmentation, speed estimator, tracking etc. 

Also now it is based on 

now support:
- [x] LIDAR Detection: pointpillar (onnx) fork from [Nvidia Cuda Pointpillars](https://github.com/NVIDIA-AI-IOT/CUDA-PointPillars)
- [x] LIDAR Ground Segmentation: [Fast lidar ground segmentation](https://ieeexplore.ieee.org/document/7989591) 

coming soon:
- [ ] LIDAR Tracking 
- [ ] LIDAR Detection support mmdetection3d/openpcdet 
- [ ] LIDAR Ground to gridmap 


## how to use 
You can start this project very easily with one launch file.
```bash
roslaunch lidar_perception pointpillars_detection.launch
```

Explanation of this launch file
```yaml
<launch>
    <node pkg="rviz"  type="rviz"  name="rviz"  args="-d $(find lidar_perception)/rviz/detection.rviz"></node>
    
    <!-- subscrible pcd topic name -->
    <param name="cloud_topic" type="string" value="/sensor/lidar_concat/points" />
    <!-- <param name="cloud_topic" type="string" value="/rslidar/helios/points" /> -->
    <node pkg="tf2_ros" type="static_transform_publisher" name="odom_to_helios" args="1.0 0 1.131 0 0 0 1 odom rslidar/helios" />
    <!-- save sub pcds -->
    <param name="dump_pcd" type="bool" value="false" />
    <param name="dump_pcd_path" type="string" value="$(find lidar_perception)/PCDS" />
    <param name="dump_time_diff" type="double" value="5" />

    <param name="is_performance" type="bool" value="true" />

    <param name="detector_model_path" type="string" value="$(find lidar_perception)/onnxs/pointpillar.onnx" />

    <!-- visulization -->
    <param name="is_vis" type="bool" value="true" />
    <param name="vis_obstacles_topic" type="string" value="/vis/pointpillars/obstacles" />

    <!-- run main lidar detection node -->
    <node pkg="lidar_perception"  type="lidar_objects_detection_node"  name="lidar_objects_detection"  output="screen"></node>
    
    <!-- run main ground segmentation node-->
    <param name="cloud_topic" type="string" value="/sensor/lidar_concat/points" />
    <param name="ground_cloud_topic" type="string" value="/sensor/ground/points" />
    <param name="filted_cloud_topic" type="string" value="/sensor/noground/points" />
    <node pkg="lidar_perception"  type="lidar_ground_filter_node"  name="lidar_ground_segmentation"  output="screen"></node>

    <!-- rosbag plugin -->
    <node pkg="rosbag" type="play" name="rosbag" args="/home/pc/data/sample/20220525_outside_samples_lidar_concated_2022-05-29-14-59-20.bag"/>
</launch>
```
