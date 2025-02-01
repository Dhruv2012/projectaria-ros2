# ROS2 for Aria

This repository contains the ROS setup for Project Aria glasses in **EgoMimic: Scaling Imitation Learning via Egocentric Videos**. 

**Useful Links**  
- [Project Page](https://egomimic.github.io/)


---

### Software Support

- :white_check_mark: **Supported Operating System**: Ubuntu 22.04
- :white_check_mark: **ROS Version**: ROS 2 Humble


## Structure
- [``eve``](./eve/): Python package providing useful classes related to Aria and visualization.  Contains scripts to use Aria glasses with ROS node (`stream_aria_ros.py`).
- [``launch``](./launch): a ROS 2 launch file for streaming aria camera.


## Install instructions
Build all of this without a conda environment in `~/aria_ros2_ws`
1. Set up ROS2. Then create a workspace dir inside your home `~/aria_ros2_ws/src` then git clone this repo inside `~/aria_ros2_ws/src`


2. Setup Aria Glasses [Aria Instructions](https://facebookresearch.github.io/projectaria_tools/docs/ARK/sdk/setup).  In brief you just need to run
```
python3 -m pip install projectaria_client_sdk --no-cache-dir
aria-doctor
aria auth pair # (then open mobile app and approve)
```
3. Install extra packages
```
pip install opencv-python fastplotlib==0.1.0a13 glfw==2.6.2 pygfx==0.1.14 dm_env h5py ipython
```
4. Add permissions on video cams
```
sudo usermod -a <user> -G video
```
5. Add aliases to `~/.bash_aliases`
```
alias setup_eve="source /opt/ros/humble/setup.bash && source ~/aria_ros2_ws/install/setup.bash"
alias ros_eve="ros2 launch eve aria.launch.py"
```
6. Restart machine


### Tips
- Use a standard terminal instead of vscode.
- Any time changes are made to the ROS package, you must run `colcon build` in `~/aria_ros2_ws`. 
- ros2 run rqt_reconfigure rqt_reconfigure


## Usage
To start up ROS run
```
setup_eve
ros_eve # (wait 15 seconds for aria to start streaming)
```

To ensure Aria stream is functional launch Rviz
```
rviz2
```

Occasionally Aria fails to connect, simply restart the computer and it should connect fine.