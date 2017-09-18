#!/bin/bash

if [ "$(uname)" != "Linux" -o "$(lsb_release -sr)" != "14.04" ]; then
  exit 0
fi

set -x

sudo apt-get remove --purge -qq -y vim vim-runtime vim-gnome vim-tiny vim-common vim-gui-common

sudo apt-get build-dep -qq -y vim-gnome

sudo apt-get install aptitude
sudo aptitude install -qq -y liblua5.1-dev luajit libluajit-5.1 python-dev ruby-dev libperl-dev mercurial libncurses5-dev libgnome2-dev libgnomeui-dev libgtk2.0-dev libatk1.0-dev libbonoboui2-dev libcairo2-dev libx11-dev libxpm-dev libxt-dev

sudo rm -rf /usr/local/share/vim

sudo rm /usr/bin/vim

sudo mkdir /usr/include/lua5.1/include
sudo mv /usr/include/lua5.1/*.h /usr/include/lua5.1/include/

sudo ln -s /usr/bin/luajit-2.0.0-beta9 /usr/bin/luajit

TMPDIR=$(mktemp -d)
cd $TMPDIR
git clone https://github.com/vim/vim.git -b v8.0.0075
cd vim/src
make distclean
./configure --with-features=huge \
    --enable-multibyte \
    --without-x \
    --enable-rubyinterp \
    --enable-largefile \
    --disable-netbeans \
    --enable-pythoninterp \
    #--with-python-config-dir=/usr/lib/python2.7/config \
    --enable-perlinterp \
    --enable-luainterp \
    --with-luajit \
    --enable-gui=auto \
    --enable-fail-if-missing \
    --with-lua-prefix=/usr/include/lua5.1 \
    --enable-cscope 
make -j
sudo make install
cd 
rm -rf $TMPDIR

sudo apt-get install -qq -y vim-gtk
