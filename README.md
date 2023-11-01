# ros2aria

# Build
```
mkdir -p ros2aria_ws/src
cd ros2aria_ws
git clone https://github.com/wust-dcr/ros2aria src/ros2aria
cd src/ros2aria
git submodule update --init
cd ../../
colcon build
source install/setup.bash
```