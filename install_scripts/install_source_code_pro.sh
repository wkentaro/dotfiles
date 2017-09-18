#!/bin/bash

if [ "`uname`" != "Linux" ]; then
  exit 0
fi

if [ -f ~/.fonts/SourceCodePro-Black.otf ]; then
  exit 0
fi

set -x

TMPDIR=$(mktemp -d)
cd $TMPDIR

wget -q https://github.com/adobe-fonts/source-code-pro/archive/2.010R-ro/1.030R-it.zip
unzip -qq 1.030R-it.zip

mkdir -p ~/.fonts
cp source-code-pro-2.010R-ro-1.030R-it/OTF/*.otf ~/.fonts/
fc-cache -f -v

rm -rf $TMPDIR

set +x