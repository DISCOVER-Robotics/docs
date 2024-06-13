# Environment Setup for AIRBOT Play

In this tutorial, we will demonstrate how to set up the software environment for AIRBOT Play.

Before proceeding, please make sure you have obtained the SDK package. If not, please contact [technical support](mailto:contact@discover-robotics.com) for further assistance. After extracting the SDK package, the directory structure is as follows:
```shell
# tree 2.8.3/
.
├── CHANGELOG
├── SHA1.txt
├── firmwares
│   ├── BLDC_4_ChibiOS.bin                                   # Motor controller firmware
│   ├── arm-interface-board-base.bin                         # Base interface board firmware 
│   └── end                                                  # End interface board firmware for different end effectors
│       ├── arm-interface-board-end_desheng_servo.bin
│       ├── arm-interface-board-end_jodell_2_fingers.bin
│       ├── arm-interface-board-end_jodell_suction_cup.bin
│       └── arm-interface-board-end_yingshi_2_fingers.bin
└── packages
    ├── airbot_aloha_2.8.3-291bbb4_amd64.deb                 # Data collection example package, depends on the basic control library package
    ├── airbot_play_2.8.3-a117c2fe_amd64.deb                 # Basic control library package, provides support for various interfaces
    ├── airbot_tools_2.8.3-291bbb4_amd64.deb                 # Example tools package, depends on the basic control library package
    ├── ros-noetic-ros-interface_2.8.3-0focal_amd64.deb      # ROS Interface package, depends on the basic control library package
    └── sdk-develop-python.zip
```

!!! question "Support for arm64(aarch64)"
	The standard SDK package includes packages for x86_64(amd64) architecture only. If you are using an ARM-based system, please contact [technical support](mailto:contact@discover-robotics.com) for further assistance.

!!! tip "TL;DR: I want to install all packages at once!"
	To install all available packages, run the following commands:
	```shell
	sudo apt install -y unzip
	sudo apt install ./packages/airbot_play_<version>_amd64.deb
	sudo apt install ./packages/airbot_tools_<version>_amd64.deb
	sudo apt install ./packages/airbot_aloha_<version>_amd64.deb
	sudo apt install ./packages/ros-noetic-ros-interface_<version>_amd64.deb
	unzip sdk-develop-python.zip && cd sdk-develop-python && sudo python3 setup.py install
	```
	Replace `<version>` with the actual version number.



## Control Library `airbot_play`

!!! note
	`airbot_play` is a basic control library package that provides support for various interfaces. It is a dependency for other packages and tools.

	With `airbot_play` installed, you can use the following tools and interfaces:

	* `airbot_read_params`: Read the status and parameters from AIRBOT Play
	* `airbot_set_zero`: Calibrate the zero position of the robot

	Detailed usage of these tools can be found in the [tools tutorial](tools.md).

To install `airbot_play`, run the following command (replace `<version>` with the actual version number):
```shell
sudo apt install ./packages/airbot_play_<version>_amd64.deb
```

Run the following command to validate the installation:
```shell
airbot_read_params -v
# Example output:
# 2.8.3-4f625187
```


## Example Tools `airbot_tools`

!!! note 
	`airbot_tools` is an example tools package that depends on the basic control library package `airbot_play`. It contains several tools for AIRBOT Play, for quick testing and debugging. The tools include:

	* `airbot_kbd_ctrl`: An example toolkit to control AIRBOT Play using the keyboard
	* `airbot_sync`: Launch two AIRBOT Play arms in synchronization mode, making one following another

	Detailed usage of these tools can be found in the [tools tutorial](tools.md).

To install `airbot_tools`, run the following command (replace `<version>` with the actual version number):
```shell
sudo apt install ./packages/airbot_tools_<version>_amd64.deb
```

Run the following command to validate the installation:
```shell
airbot_kbd_ctrl -v
# Example output:
# 2.8.3-4f625187
```

## ROS Interface `ros_interface`

!!! note
	`ros_interface` is a ROS 1 interface package that depends on the basic control library package `airbot_play`. It provides ROS 1 support for AIRBOT Play, allowing you to control the robot using ROS 1 Noetic.

To install `ros_interface`, run the following command (replace `<version>` with the actual version number):
```shell
sudo apt install ./packages/ros-noetic-ros-interface_<version>_amd64.deb
```

## Python Interface `airbot`

!!! note
	`airbot` is a Python package that provides support for AIRBOT Play. It is a `pybind11` wrapper for the basic control library package `airbot_play`.

To install `airbot`, run the following commands:
```shell
unzip sdk-develop-python.zip
cd sdk-develop-python
sudo python3 setup.py install
```