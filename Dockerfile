FROM ubuntu:trusty

RUN \
  useradd wkentaro && \
  echo wkentaro:wkentaro | chpasswd && \
  adduser wkentaro sudo && \
  su - wkentaro && \
  mkdir -p /home/wkentaro

RUN \
  set -x && \
  sudo -H apt-get update -qq && \
  sudo -H apt-get upgrade -qq -y && \
  sudo -H apt-get install -qq -y \
    git \
    fontconfig \
    python \
    python-yaml \
    unzip \
    vim \
    wget \
    zsh

RUN \
  cd ~ && \
  git clone --recursive \
    https://github.com/wkentaro/dotfiles.git .dotfiles

RUN \
  mkdir -p ~/Downloads && \
  cd ~/.dotfiles && \
  ./install.py && \
  cd ~

RUN \
  echo 'source $HOME/.zshrc.wkentaro' > ~/.zshrc && \
  exec zsh --login

RUN \
  sudo locale-gen 'en_US.UTF-8'
