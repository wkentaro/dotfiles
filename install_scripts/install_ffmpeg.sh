
#!/bin/sh

if [ "$(uname)" != "Linux" ]; then
  exit 1
fi

if which ffmpeg &>/dev/null; then
  exit 1
fi

set -x

sudo add-apt-repository ppa:jonathonf/ffmpeg-3
if [ "$(lsb_release -cs)" = "trusty" ]; then
  sudo add-apt-repository ppa:jonathonf/tesseract
fi
sudo apt-get update -qq
sudo apt-get upgrade -qq -y
sudo apt-get install -qq -y ffmpeg

set +x
