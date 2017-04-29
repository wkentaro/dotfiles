#!/bin/bash

if [ "$(uname)" != "Linux" ]; then
  exit 1
fi

sudo apt-get update -qq
sudo apt-get install -qq -y git-core curl zlib1g-dev build-essential libssl-dev libreadline-dev libyaml-dev libsqlite3-dev sqlite3 libxml2-dev libxslt1-dev libcurl4-openssl-dev python-software-properties libffi-dev

TMPDIR=$(mktemp -d)
cd $TMPDIR

wget -q http://ftp.ruby-lang.org/pub/ruby/2.2/ruby-2.2.3.tar.gz
tar -zxf ruby-2.2.3.tar.gz
cd ruby-2.2.3

./configure
make
sudo make install

ruby -v

rm -rf $TMPDIR
