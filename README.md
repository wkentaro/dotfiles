<h1 align="center">
  dotfiles
</h1>

<h4 align="center">
  My dotfiles.
</h4>

<div align="center">
  <a href="https://github.com/wkentaro/dotfiles/actions"><img src="https://github.com/wkentaro/dotfiles/workflows/CI/badge.svg"></a>
</div>

<br/>

<div align="center">
  <img src=".readme/screencast.gif" width="400px" />
</div>


## Usage

```bash
INSTALL_DIR=~/ghq/github.com/wkentaro/dotfiles  # you can change this location
git clone --recursive https://github.com/wkentaro/dotfiles.git $INSTALL_DIR && cd $INSTALL_DIR
make install
```



## CLIProxyAPI quota

`cliproxy-quota` shows remaining Claude and Codex quota for every account in
`~/.cli-proxy-api`. It requires Python 3.9 or newer and is installed by
`make install` with the other commands in `local/bin`.

```bash
cliproxy-quota
watch -n 300 cliproxy-quota
cliproxy-quota --auth-dir /path/to/credentials --timeout 15
```

Each account gets remaining-percentage bars and local reset times. Claude
shows five-hour, weekly, and any reported model-specific weekly limits.
Codex labels its primary and secondary windows by their actual duration.
Missing limits stay unreported, not 100% available. Code-review allowances,
paid credits, and extra usage are not included.

The command only reads credentials and queries provider usage endpoints. It
never refreshes tokens or changes credential files. Disabled accounts are
listed without querying them. Errors affect only that account; the command
exits with status 1 if any account fails or no accounts are found.

Output includes account emails and organization names. Keep it private.
Do not commit credential files or live output. If authentication expires,
re-login through CLIProxyAPI and run the command again.


## macOS

### iTerm2

* Download iTerm color schemes from https://github.com/mbadolato/iTerm2-Color-Schemes.
* `Iterm2 > Preferences > Profiles > Colors > Load Presets > __Dark Pastel__`.


## Ubuntu 16.04

### Input Method

* Select `fcitx` for `Keyboard input method system` in `Language Support`.
* Logout + login.
* Open fcitx settings and configure as below:

![](.readme/xenial/input_method_001.jpg)
![](.readme/xenial/input_method_002.jpg)
