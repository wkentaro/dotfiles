echo "Please edit this file."
echo "TODO(wkentaro): automate this."
exit 1

# xenial
profile_path=/org/gnome/terminal/legacy/profiles:/:b1dcc9dd-5262-4d8d-a863-c897e6d979b9
dconf write $profile_path/palette "#002731:#D01B24:#6BBE6C:#A57705:#2075C7:#C61B6E:#259185:#E9E2CB:#006388:#F4153B:#50EE84:#B17E28:#178DC7:#E14D8E:#00B29E:#FCF4DC"
dconf write $profile_path/bold-color "'#9BC1C2'"
dconf write $profile_path/background-color "'#001E26'"
dconf write $profile_path/foreground-color "'#B4D4D2'"

# trusty
gconftool-2 --set /apps/gnome-terminal/profiles/Default/foreground_color --type string "#9BC1C2"
gconftool-2 --set /apps/gnome-terminal/profiles/Default/background_color --type string "#001E26"
gconftool-2 --set /apps/gnome-terminal/profiles/Default/bold_color --type string "#B4D4D2"
gconftool-2 --set /apps/gnome-terminal/profiles/Default/palette --type string "#002731:#D01B24:#6BBE6C:#A57705:#2075C7:#C61B6E:#259185:#E9E2CB:#006388:#F4153B:#50EE84:#B17E28:#178DC7:#E14D8E:#00B29E:#FCF4DC"
