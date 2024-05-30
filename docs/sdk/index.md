# SDK for AIRBOT Play (Pro)

After extracting the firmware package, the directory structure is as follows:
```shell
.
├── CHANGELOG
├── SHA1.txt
├── firmwares
│   ├── BLDC_4_ChibiOS.bin                                   # ESC firmware
│   ├── arm-interface-board-base.bin                         # Base interface board firmware 
│   └── end                                                  # End interface board firmware for different end effectors
│       ├── arm-interface-board-end_desheng_servo.bin
│       ├── arm-interface-board-end_jodell_2_fingers.bin
│       ├── arm-interface-board-end_jodell_suction_cup.bin
│       └── arm-interface-board-end_yingshi_2_fingers.bin
└── packages
    ├── airbot_aloha_2.8.3-291bbb4_amd64.deb                 # ALOHA example package, depends on the basic control library package
    ├── airbot_play_2.8.3-a117c2fe_amd64.deb                 # Basic control library package, provides support for various interfaces
    ├── airbot_tools_2.8.3-291bbb4_amd64.deb                 # Example tools package, depends on the basic control library package
    ├── ros-noetic-ros-interface_2.8.3-0focal_amd64.deb      # ROS Interface package, depends on the basic control library package
    └── sdk-develop-python.zip
```



The AIRBOT Play supports provide SDK for:

1. C++ 
2. Python (3.8, *documents in progress*)
3. ROS 1 Noetic
4. ROS 2 Humble (*In development*)

Also, two example projects are provided, `airbot_tools` and `airbot_aloha`. 
The two projects are CMake projects built on top of C++ SDK.

1. `airbot_tools`: A set of useful tools for the use of AIRBOT Play
    * `airbot_kbd_ctrl`: Basic keyboard control for AIRBOT Play
    * `airbot_sync`: Control two AIRBOT Play simultaneously, with one following the 
    * `airbot_set_zero`: Set the current position as the zero position
  
2. `airbot_aloha`: Tools for collecting data for ALOHA algorithm training
    * `airbot_demonstrate`: Demostration for two AIRBOT Play (1 leader, 1 follower)
    * `airbot_demonstrate_dual`: Demostration for four AIRBOT Play (2 leaders, 2 followers)  

Detailed usage can be found by `--help` option in each tool. They will also be documented here in the future.