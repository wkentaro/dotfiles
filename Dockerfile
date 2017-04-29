FROM ubuntu:trusty

RUN \
  useradd wkentaro && \
  echo wkentaro:wkentaro | chpasswd && \
  mkdir -p /home/wkentaro && \
  chown -R wkentaro:wkentaro /home/wkentaro && \
  adduser wkentaro sudo

RUN \
  apt-get update -qq && \
  apt-get upgrade -qq -y && \
  apt-get install -qq -y \
    git \
    fontconfig \
    python \
    python-yaml \
    unzip \
    vim \
    wget \
    zsh

RUN \
  locale-gen 'en_US.UTF-8'

USER wkentaro

RUN \
  git clone -q --recursive \
    https://github.com/wkentaro/dotfiles.git ~/.dotfiles && \
  cd ~/.dotfiles && \
  ./install.py && \
  echo 'source $HOME/.zshrc.wkentaro' > ~/.zshrc

USER root

RUN \
  apt-get install -qq -y python-setuptools && \
  easy_install -q pip && \
  pip install -q -U pip setuptools && \
  pip install -q percol
