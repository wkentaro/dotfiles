#!/bin/bash

if which fzy &>/dev/null; then
  exit 0
fi

set -x

TMPDIR=$(mktemp -d)

cd $TMPDIR
VERSION=0.9
wget -q https://github.com/jhawthorn/fzy/archive/${VERSION}.tar.gz
tar zxf ${VERSION}.tar.gz
cd fzy-${VERSION}

make PREFIX=$HOME/.local
make install DESTDIR= PREFIX=$HOME/.local

set +x
