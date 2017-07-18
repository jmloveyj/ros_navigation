#!/bin/sh
export GAZEBO_MODEL_PATH=`rospack find custom_description`/description/meshes/
roslaunch custom_description sim_sweeper.launch
