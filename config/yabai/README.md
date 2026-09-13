# yabai

yabai owns automatic tiling; keep Paneru and its border watcher stopped.
Configuration is linked to `~/.config/yabai/yabairc`; shortcuts live in
`config/skhd/skhdrc`. Start with `yabai --start-service` and
`skhd --start-service`. Grant Accessibility access if macOS requests it.

The border watcher starts with yabai and marks the focused window with an
8-point JankyBorders outline: light blue (`#89b4fa`) when tiled, darker blue
(`#0a84ff`) when floating or while all tiling is paused. Inactive borders are
transparent. Color updates within about 0.2 seconds; unavailable window state
hides the border. Check color selection with `local/bin/watch-yabai-borders --check`.

Floating exceptions: Labelme (including Python-launched windows titled
`Labelme` or `Labelme - …`), System Settings, 1Password, Calculator, Activity
Monitor, AppCleaner, Karabiner-Elements, Karabiner-EventViewer, and BetterTouchTool.
Standard dialogs are already excluded by yabai's normal window eligibility rules.

| Shortcut | Action |
| --- | --- |
| Option + arrows | Focus window |
| Option + Shift + arrows | Swap windows |
| Option + 1–5 | Focus existing macOS Space |
| Option + Shift + 1–5 | Send window to Space without following |
| Control + Option + Escape | Toggle all Spaces between BSP and floating |
| Option + F | Zoom tiled window within current Space |
| Control + Command + equal / minus | Grow / shrink window horizontally |
| Option + Return | New terminal window |
| Option + Shift + Return | New browser window |

The single-window floating shortcut is currently disabled in the skhd config.

All Spaces inherit the global layout. Do not add per-Space layout overrides:
those bypass the global toggle. Pausing preserves current window positions;
resuming rebuilds the BSP arrangement, preserving individually floated windows.
New Spaces inherit the paused layout too. Restarting yabai resumes BSP tiling.

The numbered shortcuts target existing Spaces; create additional desktops in
Mission Control as needed. Disable “Automatically rearrange Spaces based on
most recent use” in macOS settings to keep numbers stable. SIP remains enabled;
workspace behavior depends on the installed yabai/macOS version.

Run `sh -n config/yabai/yabairc local/bin/toggle-yabai-tiling` for syntax checks.
With yabai running, `local/bin/toggle-yabai-tiling --check` exercises a live
pause/resume round trip. This rebuilds the current tiled arrangement.
