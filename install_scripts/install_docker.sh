#!/bin/sh

# docker
# ~~~~~~


if [ "`uname`" != "Linux" ]; then
  exit 1
fi


sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 36A1D7869245C8950F966E92D8576A8BA88D21E9
sudo sh -c "echo deb https://get.docker.com/ubuntu docker main > /etc/apt/sources.list.d/docker.list"
sudo apt-get update -qq -y

sudo apt-get install -qq -y lxc-docker

# change owner of docker.sock
sudo chown $(whoami) /var/run/docker.sock

# install using dockers
docker pull ubuntu
