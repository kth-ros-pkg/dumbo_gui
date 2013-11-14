dumbo_gui
=========

Overview
---------------------------------------------

GUI for controlling CVAP's Dumbo robot


Installing
---------------------------------------------

Install **Qt** libraries:
    sudo apt-get install libqt4-dev qt4-dev-tools

It might also be useful to install the Qtcreator IDE:
    sudo apt-get install qtcreator

Download the code from the `groovy` or `hydro` branch in the  github repository to your catkin workspace and compile using catkin:

    cd ~/catkin_ws/src
    git clone -b groovy https://github.com/kth-ros-pkg/dumbo_gui.git
    cd ~/catkin_ws && catkin_make
    


Running dumbo_dashboard
---------------------------------------------

Just rosrun the dumbo_dashboard binary:
    
    rosrun dumbo_dashboard dumbo_dashboard