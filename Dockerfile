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
    aptitude \
    ctags \
    git \
    fontconfig \
    python \
    python-setuptools \
    python-yaml \
    unzip \
    vim \
    wget \
    zsh

RUN \
  easy_install -q pip && \
  pip install -q -U pip setuptools && \
  pip install -q percol

RUN \
  locale-gen 'en_US.UTF-8' && \
  chsh -s /bin/zsh wkentaro

USER wkentaro

RUN \
  git clone -q --recursive \
    https://github.com/wkentaro/dotfiles.git ~/.dotfiles && \
  cd ~/.dotfiles && \
  ./install.py && \
  echo 'source $HOME/.zshrc.wkentaro' > ~/.zshrc

USER root

RUN \
  cd /home/wkentaro/.dotfiles/install_scripts && \
  bash ./install_ffmpeg_trusty.sh && \
  bash ./install_hub.sh && \
  bash ./install_ros_indigo.sh && \
  bash ./install_ruby.sh && \
  bash ./install_timg.sh && \
  bash ./install_tmux.sh && \
  bash ./install_ubuntu_tools.sh && \
  bash ./install_vim_with_sudo.sh && \
  bash ./install_zsh.sh

RUN \
  chsh -s /usr/local/bin/zsh wkentaro
