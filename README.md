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
- power board can frames structure and format.........................................TODO
- functions tested, working...........................................................OK
- robot joints order and direction....................................................OK
- imu directions and test.............................................................TODO
- first standup test..................................................................TODO
