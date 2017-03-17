#!/bin/sh

if pip show util &>/dev/null; then
  exit 0
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR

git clone https://github.com/wkentaro/pyutil.git
cd pyutil
python setup.py install --user --prefix=

rm -rf $TMPDIR
