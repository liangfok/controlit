# ControlIt! - A Whole Body Operational Space Control Middleware #

## Introduction ##

This is stack contains ControlIt!, a middleware that supports the Whole Body
Operational Space Control algorithm. It employs a plugin-based modular
software architecture so other whole body control algorithms could be
supported as well.

## Installation ##

### Create a ROS Catkin Workspace ###

This step is only necessary if you do not already have a ROS Catkin workspace that you want to use for ControlIt! development. The commands below create a workspace called "controlit_workspace". Feel free to customize it to another name.

    $ source /opt/ros/indigo/setup.bash
    $ mkdir -p ~/controlit_workspace/src
    $ cd ~/controlit_workspace/src
    $ catkin_init_workspace
    $ cd ..
    $ catkin_make

Edit ~/.bashrc and add the following line at the bottom:

    source $HOME/controlit_workspace/devel/setup.bash