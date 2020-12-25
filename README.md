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

## Usage

1. Launch LIDAR odometry and SLAM
```
ros2 launch pure_lidarslam_toolkit lidar_odometry.launch.py 
```

2. Launch map to mesh converter
```
ros2 launch pure_lidarslam_toolkit map_to_mesh.launch.py 
```
