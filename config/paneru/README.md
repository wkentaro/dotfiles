# Paneru

Paneru 0.5.0 manages windows in scrolling columns. Configuration lives in
`~/.config/paneru/init.lua`; saving it reloads the configuration. The Lua file
replaces TOML configuration.

| Shortcut | Action |
| --- | --- |
| Alt + Return / Alt + Shift + Return | New WezTerm / Brave window |
| Alt + Left / Right | Focus previous / next column |
| Alt + Shift + Left / Right | Swap with previous / next column |
| Alt + T | Toggle one window between tiled and floating |
| Alt + Shift + T | Toggle floating mode for the current workspace |
| Alt + F | Toggle full width |
| Alt + Shift + 0 | Center window |
| Alt + Shift + equal / minus | Grow / shrink through preset widths |
| Alt + 1–5 | Switch Paneru virtual workspace |
| Alt + Shift + 1–5 | Move window and follow |
| Alt + scroll | Scroll columns |
| Alt + Shift + scroll | Switch virtual workspaces |
| Cmd + Ctrl + grave | Focus a visible window on another display |
| Ctrl + Alt + Q | Quit Paneru |

Workspace floating mode includes newly opened windows. Returning to tiling
restores only the windows released by that mode, preserving individually
floated windows and the Labelme/System Settings exceptions. Mode state survives
configuration reloads. Paneru stays running in both modes; there is no global
stop/start shortcut.

Floating windows are shared across Paneru virtual workspaces within a native
macOS Space. They are not isolated like tiled windows. Returning to tiling
does not restore exact column widths or ordering.

Windows have 5px padding on each side, giving 10px gaps between neighbors.
JankyBorders draws 8-point borders: tiled `#89b4fa`, floating `#0a84ff`, inactive
transparent. The border watcher updates colors within about 0.2 seconds and
hides borders when Paneru is unavailable. Native Paneru borders are disabled.

## Installation and services

Install the release with Lua support, then link the files listed in
`dotfiles.yaml` (including the border watcher in `local/bin`):

```sh
cargo install --git https://github.com/karinushka/paneru --tag v0.5.0 --locked
"$HOME/.cargo/bin/paneru" install
"$HOME/.cargo/bin/paneru" start
skhd --start-service
launchctl bootstrap "gui/$(id -u)" "$HOME/Library/LaunchAgents/com.wkentaro.paneru-borders.plist"
```

The Paneru service uses the Cargo-installed binary. JankyBorders and Python 3
are installed through Homebrew. Stop any manually launched Paneru, skhd, or
JankyBorders process before starting their services. Grant the Paneru binary
Accessibility access if prompted. The yabai package can remain installed, but
its service and configuration are removed.

```sh
"$HOME/.cargo/bin/paneru" stop
"$HOME/.cargo/bin/paneru" start
```

For multiple monitors, Paneru recommends a vertical arrangement in macOS
Display settings to prevent offscreen windows migrating to adjacent displays.
Small window slivers at screen edges are expected.

## Checks

```sh
lua config/paneru/float-workspace.lua --check
local/bin/watch-paneru-borders --check
```

Reference: [Paneru 0.5.0 scripting](https://github.com/karinushka/paneru/blob/v0.5.0/SCRIPTING.md).
