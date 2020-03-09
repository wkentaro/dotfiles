FROM ubuntu:xenial

ENV DEBIAN_FRONTEND noninteractive

RUN \
  apt-get update -qq && \
  apt-get upgrade -qq -y && \
  apt-get install -qq -y \
    apt-transport-https \
    aptitude \
    build-essential \
    ctags \
    curl \
    fontconfig \
    git \
    locales \
    lsb-release \
    python-dev \
    python-pip \
    python-setuptools \
    python-yaml \
    python3-setuptools \
    python3-pip \
    software-properties-common \
    sudo \
    unzip \
    vim \
    wget \
    zsh

RUN \
  export LANG=en_US.UTF-8 && \
  export LC_ALL=$LANG && \
  locale-gen --purge $LANG

RUN \
  useradd wkentaro && \
  echo "wkentaro ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/wkentaro && \
  chmod 0440 /etc/sudoers.d/wkentaro && \
  mkdir -p /home/wkentaro && \
  chown wkentaro:wkentaro /home/wkentaro && \
  chsh -s /usr/bin/zsh wkentaro

COPY . /home/wkentaro/.dotfiles/

RUN \
  chown -R wkentaro:wkentaro /home/wkentaro/.dotfiles

USER wkentaro
WORKDIR /home/wkentaro

RUN \
  cd ~/.dotfiles && \
  ./install.py --sudo && \
  echo 'source $HOME/.zshrc.wkentaro' > ~/.zshrc

USER wkentaro
ENV SHELL /usr/bin/zsh
CMD /usr/bin/zsh
