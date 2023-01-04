# WoLF - Ylo2 hardware interface

Hardware Abstraction Layer for ylo2 robot to be used with WoLF: Whole Whole-body Locomotion Framework
available [here](https://github.com/graiola/wolf-setup).
Ylo2 robot official repository is available [here](https://github.com/elpimous/ylo-2).

status :
--------
- robot motors controller lib => moteus_driver (using a Peak M2 CanFD 4 ports board)..OK
- robot zeroing.......................................................................OK
- read values from Moteus are in turn.  Needed to convert to radians..................OK
- joints can frames (send and receive) structure and format...........................OK
- power board can frames structure and format.........................................OK
- functions tested, working...........................................................OK
- robot joints order and direction....................................................OK
- imu directions and test.............................................................OK
- first standup test..................................................................TODO


Commands :
--------
- run the ylo2 wolf controller : 
roslaunch wolf_ylo2_interface run.launch

- run the wolf controller (interacts with wolf_ylo2_interface) : 
roslaunch wolf_controller wolf_controller_bringup.launch input_device:=keyboard robot_name:=ylo2 gazebo:=false

- run wolf simulation (without real robot) : 
roslaunch wolf_controller wolf_controller_bringup.launch input_device:=keyboard robot_name:=ylo2

- same with full gui : 
roslaunch wolf_controller wolf_controller_bringup.launch input_device:=keyboard robot_name:=ylo2 full_gui:=true

- ask moteus gui :
python3 -m moteus_gui.tview --devices=1-3 --can-iface pcan --can-chan PCAN_PCIBUS1

- motor calibration :
/home/ylo2/Documents/Mjbots_tools/python_code/working/Qdd100_calibration_tool.py
