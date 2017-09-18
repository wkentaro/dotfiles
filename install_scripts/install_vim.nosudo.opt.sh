#!/bin/sh

if [ "$(uname)" != "Linux" ]; then
  echo "[$(basename $0)] Unsupported platform: $(uname)"
  exit 0
fi

set -x

TMPDIR=$(mktemp -d)
cd $TMPDIR

git clone https://github.com/vim/vim.git -b v8.0.0075
cd vim/src

make distclean

./configure --with-features=huge \
            --enable-multibyte \
            --enable-rubyinterp=yes \
            --enable-pythoninterp=yes \
            --with-python-config-dir=/usr/lib/python2.7/config \
            --enable-python3interp=yes \
            --with-python3-config-dir=/usr/lib/python3.5/config \
            --enable-perlinterp=yes \
            --enable-luainterp=yes \
            --enable-gui=gtk2 --enable-cscope --prefix=/usr

make -j VIMRUNTIMEDIR=~/.local/share/vim/vim80

make install prefix=~/.local

rm -rf $TMPDIR

set +x
