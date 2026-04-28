#!/bin/sh

export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/user/workspaces/beta_ws/src/aeroloop_gazebo/plugins/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=/home/user/workspaces/beta_ws/src/aeroloop_gazebo/worlds:/home/user/workspaces/beta_ws/src/aeroloop_gazebo/models:$GZ_SIM_RESOURCE_PATH
gz sim -r -v 1 betaloop_iris_betaflight_demo_harmonic.sdf