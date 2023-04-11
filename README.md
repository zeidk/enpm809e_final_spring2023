# enpm809e_final_spring2023
Packages for assignments and the final project

## Installation

```bash
source /opt/ros/galactic/setup.bash
cd
mkdir -p ~/final_809e_ws/src
cd final_809e_ws/src
git clone https://github.com/zeidk/enpm809e_final_spring2023.git
cd ..
sudo apt install python3-rosdep
sudo rosdep init
rosdep update --include-eol-distros
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
ros2 launch ariac_gazebo ariac.launch.py trial_name:=human_kitting
```
