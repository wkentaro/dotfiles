<img src=".readme/screencast.gif" align="right" width="365" />

# dotfiles

[![Build Status](https://travis-ci.org/wkentaro/dotfiles.svg?branch=master)](https://travis-ci.org/wkentaro/dotfiles)
[![Docker Build Status](https://img.shields.io/docker/build/wkentaro/dotfiles.svg)](https://hub.docker.com/r/wkentaro/dotfiles/)

My dotfiles.


## Usage

```bash
INSTALL_DIR=$HOME/.dotfiles  # you can change this location
git clone --recursive https://github.com/wkentaro/dotfiles.git $INSTALL_DIR && cd $INSTALL_DIR
./install.py
```


## Docker

You can try my shell env in docker.

```bash
docker run -it wkentaro/dotfiles
```


## macOS

### iTerm2

* Download iTerm color schemes from https://github.com/mbadolato/iTerm2-Color-Schemes.
* `Iterm2 > Preferences > Profiles > Colors > Load Presets > __Dark Pastel__`.


## Ubuntu 16.04

* Select `fcitx` for `Keyboard input method system` in `Language Support`.
* Logout + login.
* Open mozc tool, and load keyboard settings `config/mozc_keymap_for_win_keyboard.txt`.
