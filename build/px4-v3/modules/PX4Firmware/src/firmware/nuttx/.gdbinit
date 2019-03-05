target extended /dev/ttyACM0
monitor swdp_scan
attach 1
monitor vector_catch disable hard
set mem inaccessible-by-default off
set print pretty
source /home/beatrice/Licenta_FisiereComplete/ardupilot/modules/PX4Firmware/Debug/PX4
