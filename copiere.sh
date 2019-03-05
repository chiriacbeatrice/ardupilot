#!/bin/sh
#A script for build and copy paste the file in Windows
######
##Main body of the script starts here#####
####

 echo "Script starts here..."
 
 git clone -b betty1 --single -branch https://github.com/chiriacbeatrice/ardupilot.git
 cd ardupilot
 git submodule update --init --recursive
 Tools/scripts/install-prereqs-ubuntu.sh -y
 . ~/.profile 

 echo "End of script..."

