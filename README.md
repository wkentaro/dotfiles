# dotfiles

My dotfiles.


## Usage

```bash
cd ~
git clone --recursive https://github.com/wkentaro/dotfiles.git
cd .dotfiles
./install.py
```


## Docker

You can try my shell env in docker.

```bash
docker run -u wkentaro -e SHELL=/usr/local/bin/zsh -it wkentaro/dotfiles /usr/local/bin/zsh
```


## Optional

### iTerm2

* Download iTerm color schemes from https://github.com/mbadolato/iTerm2-Color-Schemes.
* `Iterm2 > Preferences > Profiles > Colors > Load Presets > __Dark Pastel__`.
