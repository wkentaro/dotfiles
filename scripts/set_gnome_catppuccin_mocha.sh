#!/bin/bash

profile_path=/org/gnome/terminal/legacy/profiles:/:b1dcc9dd-5262-4d8d-a863-c897e6d979b9

# Catppuccin Mocha palette
# 0-7:  Surface1, Red, Green, Yellow, Blue, Pink, Teal, Subtext1
# 8-15: Surface2, Red, Green, Yellow, Blue, Pink, Teal, Text
dconf write "$profile_path/palette" "['#45475A','#F38BA8','#A6E3A1','#F9E2AF','#89B4FA','#F5C2E7','#94E2D5','#BAC2DE','#585B70','#F38BA8','#A6E3A1','#F9E2AF','#89B4FA','#F5C2E7','#94E2D5','#CDD6F4']"
dconf write "$profile_path/background-color" "'#1E1E2E'"
dconf write "$profile_path/foreground-color" "'#CDD6F4'"
dconf write "$profile_path/bold-color" "'#CDD6F4'"
dconf write "$profile_path/bold-color-same-as-fg" "true"
dconf write "$profile_path/use-theme-colors" "false"

echo "Catppuccin Mocha applied to gnome-terminal."
