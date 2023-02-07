# AutonomousMobileRoboticsHomeworks S23

## Homeworks

1. Comparison of analytical and simulated trajectories

## Setup

1. Install Gazebo

2. Install apropriate ROS2 version

3. Clone the repositories

```bash
 mkdir -p ~/ws/src
 cd ~/ws/src
 git clone https://github.com/GPrathap/autonomous_mobile_robots.git
 git clone https://github.com/ipipos56/AutonomousMobileRoboticsHW.git
```

4. Build the workspace

```bash
 cd ~/ws
 rosdep install --from-paths src --ignore-src -r -y
 colcon build
```

5. Source the workspace

```bash
 source /usr/share/gazebo/setup.sh
 source ~/ws/install/setup.bash
```

6. Lauch build

```bash
 ros2 launch hagen_gazebo hagen.launch.py
```

7. Run script

   `python3 <script_name>`

