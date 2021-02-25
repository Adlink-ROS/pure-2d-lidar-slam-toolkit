# pure-2d-lidar-slam-toolkit

This SLAM Toolkit helps users to try SLAM by using only a LiDAR and a ROScube without having a real robot. Furthermore, the toolkit is able to convert a PGM map file into a Gazebo world, give users the possiblity to do Navigation simulation according to their environment.

In general, the toolkit offers:

1. Pure LiDAR-SLAM 

2. Map to Gazebo World Conversion

We have tested the toolkit by using SICK LiDARs on ADLINK ROScube. It's possible to use different hardware components but you have to verify it on your own.

## Installation

1. Git clone slam toolkit and other related sources
    ```
    mkdir ~/pureslam_ros2_ws/src -p
    cd ~/pureslam_ros2_ws/
    wget https://raw.githubusercontent.com/adlink-ros/pure-2d-lidar-slam-toolkit/main/neuron-app.repos
    vcs import src < neuron-app.repos 

    ```

2. Install dependencies
    ```
    cd ~/pureslam_ros2_ws/
    source /opt/ros/foxy/setup.bash
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy
    pip3 install -r ~/pureslam_ros2_ws/src/pure-2d-lidar-slam-toolkit/requirement.txt
    ```

3. Colcon build the package
    ```
    cd ~/pureslam_ros2_ws/
    colcon build --symlink-install 
    source ~/pureslam_ros2_ws/install/local_setup.bash
    ```
## Parameter setting
```
vim ~/pureslam_ros2_ws/src/sick_scan2/config/sick_tim_7xxS.yaml
vim ~/pureslam_ros2_ws/src/map2gazebo/config/default.yaml 
```
### SICK Driver: sick_tim_7xxS.yaml: 

1. To change IP in launch file, please modify value of 'hostname'.

    Please check default IP of your LiDAR and follow [instuction](https://github.com/SICKAG/sick_scan2).

2. Modify value of 'frame_id' to 'base_link'
```
sick_scan2:
  ros__parameters:
    hostname : "192.168.1.2"
    frame_id : "base_link"
    scanner_name : "sick_tim_7xxS"
    port : 2112
    min_ang : -2.35619449
    max_ang : 2.35619449
```
3. Check Ethernet connection to scanner with netcat
    ```
    nc -z -v -w5 192.168.1.2 2112
    ```
### Map2gazebo: default.yaml

You can change height of boxes and map threshold. **Not recommend to modify export_dir.**
```
map2gazebo:
  ros__parameters:
    map_topic: "/map"
    # Can be "stl" or "dae"
    mesh_type: "stl"
    # Minimum threshold value for considering a map cell occupied
    occupied_thresh: 1
    # Height of boxes in gazebo environment
    box_height: 2.0

    #export_dir: 
```

## SOP for Pure LiDAR-SLAM 

1. Launch LiDAR driver, ex. for SICK LiDAR TiM781S:
    ```
    ros2 launch sick_scan2 sick_tim_7xxS.launch.py  
    ```

2. Launch LiDAR odometry and SLAM
    ```
    ros2 launch pure_lidarslam_toolkit lidar_odometry.launch.py 
    ```

## SOP for Map to Gazebo World Conversion

1. Launch map converter, there are two options to convert the map: online and offline.

    **Option 1: Online conversion**

    In this way, you can run the SLAM and convert the map at the same time.

    ```    
    ros2 launch pure_lidarslam_toolkit map_to_mesh.launch.py 
    ```
    
    The mesh file 'map.stl' will be stored at current directory.
 
    After mapping is done, save your map and turn off SLAM.

    ```
    ros2 run nav2_map_server map_saver_cli -f <map_dir>/map --ros-args -p save_map_timeout:=10000
    ```

    **Option 2: Offline conversion**

    If you already have a PGM map file, you can directly use map2gazebo_offline.py  to convert map to mesh file. Remember to set file name of map and export directory in absolute path format.

    ```
    cd ~/pureslam_ros2_ws/
    python3 src/map2gazebo/map2gazebo/map2gazebo_offline.py --map_dir /path/to/map/map.pgm --export_dir /path/to/export_dir
    ```

2. Copy the converted mesh file to neuronbot2 package and rebuild the package

    ```
    cp map.stl ~/pureslam_ros2_ws/src/pure-2d-lidar-slam-toolkit/map/meshes/

    cp -rf ~/pureslam_ros2_ws/src/pure-2d-lidar-slam-toolkit/map ~/pureslam_ros2_ws/src/neuronbot2/neuronbot2_gazebo/models/

    cp ~/pureslam_ros2_ws/src/pure-2d-lidar-slam-toolkit/worlds/map.model ~/pureslam_ros2_ws/src/neuronbot2/neuronbot2_gazebo/worlds/

    colcon build --symlink-install
    ```
3. Launch Gazebo simulation to verify the result

    ```
    source ~/pureslam_ros2_ws/install/local_setup.bash
    ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=map.model
    ```
    



