# Interbotix ALOHA

Project Websites:

* [ALOHA](https://tonyzhaozh.github.io/aloha/)
* [Mobile ALOHA](https://mobile-aloha.github.io/)

Trossen Robotics Documentation: https://docs.trossenrobotics.com/aloha_docs/

This codebase is forked from the [Mobile ALOHA repo](https://github.com/MarkFzp/mobile-aloha), and contains teleoperation and dataset collection and evaluation tools for the Stationary and Mobile ALOHA kits available from Trossen Robotics.

To get started with your ALOHA kit, follow the [ALOHA Getting Started Documentation](https://docs.trossenrobotics.com/aloha_docs/getting_started.html).

To train imitation learning algorithms, you would also need to install:

* [ACT for Stationary ALOHA](https://github.com/Interbotix/act).
* [ACT++ for Mobile ALOHA](https://github.com/Interbotix/act-plus-plus)

# Structure
- [``aloha``](./aloha/): Python package providing useful classes and constants for teleoperation and dataset collection.
- [``config``](./config/): a config for each robot, designating the port they should bind to, more details in quick start guide.
- [``launch``](./launch): a ROS 2 launch file for all cameras and manipulators.
- [``scripts``](./scripts/): Python scripts for teleop and data collection


# instructions
Build all of this in root, no conda env
1. Set up ROS2 + trossen https://docs.trossenrobotics.com/aloha_docs/getting_started/stationary/software_setup.html + aloha cams
2. Aria setup https://facebookresearch.github.io/projectaria_tools/docs/ARK/sdk/setup.  Brief version here...
```
python3 -m pip install projectaria_client_sdk --no-cache-dir
aria-doctor
aria auth pair (then open mobile app and approve)
```
3. extra packages
```
pip install opencv-python fastplotlib==0.1.0a13 glfw==2.6.2 pyg==0.1.14
```
4. permissions on video cams
sudo usermod -a <user> -G video
5. aliases in ~/.bash_aliases
```
alias setup_aloha="source /opt/ros/humble/setup.bash && source ~/interbotix_ws/install/setup.bash"
alias ros_aloha="usbreset "Aria" && sleep 3 && ros2 launch aloha aloha_bringup.launch.py"
```
6. Restart machine

7. maybe build realsense from scratch, i used this commit 7c163180e56172f38700d9f3ac9a4205de03765e

To run the scripts in aloha, use the eplay conda env found in [Egoplay Repo](https://github.com/SimarKareer/EgoPlay/blob/main/environment.yaml)


- Remember to use terminal instead of vscode terminal.
- if changes are made to aloha make sure to `colcon build` in `~/interbotix_ws`

Useful commands
- ros2 run rqt_reconfigure rqt_reconfigure