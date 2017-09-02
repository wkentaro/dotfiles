FROM ubuntu:xenial

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
    build-essential \
    ctags \
    curl \
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
  echo en_US.UTF-8 UTF-8 >> /etc/locale.gen && \
  dpkg-reconfigure locales && \
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
  set -e && \
  for fname in $(command ls *.sh); do bash $fname; done && \
  set +e

RUN \
  chsh -s /usr/local/bin/zsh wkentaro

USER wkentaro
ENV HOME /home/wkentaro
ENV SHELL /usr/local/bin/zsh
CMD /usr/local/bin/zsh
