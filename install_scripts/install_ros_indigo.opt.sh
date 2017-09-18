#!/bin/bash

if ! which lsb_release >/dev/null; then
  sudo -H apt-get install -qq -y lsb-release
  sudo -H apt-get install -qq -y software-properties-common
fi

sudo sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo -H apt-get update -qq

sudo -H apt-get install -q -y ros-indigo-desktop-full

sudo rosdep init
rosdep update
