# This readme file is currently under construction

Please be aware that this readme file is a work in progress and may not be complete. We will be updating it regularly with new information, so please check back later for updates.

# Harpia - A Hybrid System for UAV Missions

## About the project

Harpia 2 is the ROS 2 version of the Harpia system for UAV mission and path planning. The project aims to provides a set of tools and libraries for generate UAV autonomous missions in a high-level for the user. 

## The Architecture

## Instalation
<aside>
💡 Make sure that the system is updated →`sudo apt-get update`
</aside>

### System Versions

- Ubuntu: Ubuntu 22.04 LTS
- ROS → Humble
- QGroundControl → 

### Dependecies

- `sudo apt install curl libc6 libstdc++6 openjdk-11-jdk python3-prettytable python3-pip python3-lxml libxml2 libxslt1.1`
- `sudo apt-get install flex bison python3-opencv python3-matplotlib libxml2 libxslt1-dev`
- `sudo apt install git`
- `sudo pip3 install git+https://github.com/colcon/colcon-common-extensions`
- `sudo -H pip3 install --upgrade pip`
- `pip install pyAgrum termcolor toml empy packaging jinja2 rospkg pandas pyproj shapely spicy scikit-learn psutil install future testresources kconfiglib jsonschema sympy==1.7.1 graphviz lxml  seaborn keras tensorflow pyspark plotly cloudpickle jupyter jupyterlab pyros-genmsg`

### ROS 2 Installation

- ROS Installation on Ubuntu → [official link](https://docs.ros.org/en/humble/Installation.html)

### MavROS Installation:

- [MAVROS documentation](https://index.ros.org/p/mavros/)
- [MAVROS installation guide](https://github.com/mavlink/mavros/tree/ros2/mavros)

### QGroundControl Installation:
- [QGroundControl site](http://qgroundcontrol.com/)
- Download the app [here](https://github.com/mavlink/qgroundcontrol/releases/download/v4.2.1/QGroundControl.AppImage)
- `sudo usermod -a -G dialout $USER`
- `sudo apt-get remove modemmanager -y`
- `sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y`
- LogOut and Login again
- `chmod +x ./QGroundControl.AppImage`
- Run `./QGroundControl.AppImage`

### PX4 Firmware:
- [PX4 Firmware site](https://docs.px4.io/main/en/)
- `git clone --depth=1 --branch v1.13.2 https://github.com/PX4/PX4-Autopilot/`
- `cd PX4-Autopilot`
- `make`
- `bash ./Tools/setup/ubuntu.sh`

### Project setup

- `git clone https://github.com/RaphaelBonaccorsi/harpia2/`

### Build

- `cd harpia2`
- `colcon build`

## Simulation Video

[![Video](https://i9.ytimg.com/vi_webp/--hn0I5QUJ8/mq2.webp?sqp=CJzBop4G-oaymwEmCMACELQB8quKqQMa8AEB-AHUBoAC4AOKAgwIABABGGQgZShUMA8=&rs=AOn4CLALXTaHg7IRncNrzhT9RfPaIgf7Pg)](https://youtu.be/--hn0I5QUJ8)

## Articles 
- [Service-Oriented Architecture to Integrate Flight Safety and Mission Management Subsystems into UAVs, ICAS, BeloHorizonte, 2018](https://www.icas.org/ICAS_ARCHIVE/ICAS2018/data/papers/ICAS2018_0374_paper.pdf)
- [Harpia: A Hybrid System for Agricultural UAV Missions](https://authors.elsevier.com/tracking/article/details.do?surname=Vannini&aid=100191&jid=ATECH)

## Implementation Schedule 

## References

### PlanSys2

