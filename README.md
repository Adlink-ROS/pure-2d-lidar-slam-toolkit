# pure-2d-lidar-slam-toolkit
## Installation

1. Git clone slam toolkit and other related sources
```
mkdir ~/slam_ros2_ws/src -p
cd slam_ros2_ws/src
git clone https://github.com/H-HChen/pure-2d-lidar-slam-toolkit.git
vcs import src < neuron-app.repos 

```

2. Install dependencies
```
cd ~/slam_ros2_ws/
source /opt/ros/foxy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro foxy
pip3 install -r ./src/pure-2d-lidar-slam-toolkit/requirement.txt
```

3. Colcon build the package
```
cd ~/slam_ros2_ws/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source ~/slam_ros2_ws/install/local_setup.bash
```
## Parameter setting
```
vim ./src/pure-2d-lidar-slam-toolkit/src/sick_scan2/config/sick_tim_7xxS.yaml
vim ./src/pure-2d-lidar-slam-toolkit/src/map2gazebo/config/default.yaml 
```
### SICK Driver: sick_tim_7xxS.yaml: 

1. To change IP in launch file, please modify value of 'hostname'.

Please check default IP of your LIDAR and follow [instuction](https://github.com/SICKAG/sick_scan2).

2. Modify value of 'frame_id' to 'base_link'
```
  ros__parameters:
    hostname : "192.168.1.2"
    frame_id : "base_link"
    scanner_name : "sick_tim_7xxS"
    port : 2112
    min_ang : -2.35619449
    max_ang : 2.35619449
```
### Map2gazebo: default.yaml
You can change height of boxes and mesh type. **Not recommend to modify export_dir.**
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
## Usage

1. Launch LIDAR driver, ex. for SICK LIDAR TiM781S:
```
ros2 launch sick_scan2 sick_tim_7xxS.launch.py  
```
If you encounter TCP failure, please check default IP of your LIDAR and follow [instuction](https://github.com/SICKAG/sick_scan2).

2. Launch LIDAR odometry and SLAM
```
ros2 launch pure_lidarslam_toolkit lidar_odometry.launch.py 
```

3. Launch map converter
```
ros2 launch pure_lidarslam_toolkit map_to_mesh.launch.py 
```
The meth file 'map.stl' will be stored at current directory.
 
4. Copy gazebo model to neuronbot2 package
```
cp map.stl ./src/pure-2d-lidar-slam-toolkit/map/meshes/
cp -rf ./src/pure-2d-lidar-slam-toolkit/map ./src/pure-2d-lidar-slam-toolkit/src/neuronbot2/neuronbot2_gazebo/models/
cp ./src/pure-2d-lidar-slam-toolkit/worlds/map.model ./src/pure-2d-lidar-slam-toolkit/src/neuronbot2/neuronbot2_gazebo/worlds/
```
5. Launch gazebo simulation
```
ros2 launch neuronbot2_gazebo neuronbot2_world.launch.py world_model:=map.model

```
