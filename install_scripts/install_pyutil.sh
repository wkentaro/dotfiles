#!/bin/bash

if pip show util &>/dev/null; then
  exit 0
fi

TMPDIR=$(mktemp -d)
cd $TMPDIR

git clone https://github.com/wkentaro/pyutil.git
cd pyutil
if [ $(uname) = Darwin ]; then
  python setup.py install
else
  python setup.py install --user
fi

rm -rf $TMPDIR
