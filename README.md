# WoLF - Ylo2 hardware interface

Hardware Abstraction Layer for ylo2 robot to be used with WoLF: Whole Whole-body Locomotion Framework
available [here](https://github.com/graiola/wolf-setup).
Ylo2 robot official repository is available [here](https://github.com/elpimous/ylo-2).

status :
--------
- robot motors controller lib => moteus_driver (using a Peak M2 CanFD 4 ports board)  OK
- robot zeroing                                                                       OK
- robot joints order from 1 to 12, FL, FR, HL, HR                                     OK
- read values from Moteus are about 360°.  Needed to convert to radians               OK                
- can frames (send and receive) structure and format                                Checked
- functions tested, working							       Checked
- robot joints directions                                                             TODO
