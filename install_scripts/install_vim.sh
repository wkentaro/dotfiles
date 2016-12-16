#!/bin/sh

TMPDIR=$(mktemp -d)

cd $TMPDIR

git clone https://github.com/vim/vim.git -b v8.0.0075
cd vim/src

make distclean

./configure \
    --with-features=huge \
    --enable-multibyte \
    --enable-rubyinterp \
    --enable-pythoninterp \
    --enable-largefile \
    --disable-netbeans \
    --with-python-config-dir=/usr/lib/python2.7/config \
    --enable-perlinterp \
    --enable-luainterp \
    --with-luajit \
    --enable-gui=auto \
    --enable-fail-if-missing \
    --with-lua-prefix=/usr/include/lua5.1 \
    --enable-cscope \
    --prefix=~/.local

make VIMRUNTIMEDIR=~/.local/share/vim/vim80

make install prefix=~/.local
