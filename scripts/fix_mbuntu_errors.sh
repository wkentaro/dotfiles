#!/bin/sh

sudo mkdir -p /usr/share/themes/MBuntu-Y-For-Unity/gtk-2.0/widgets/images/panel
cd /usr/share/themes/MBuntu-Y-For-Unity/gtk-2.0/widgets/images/panel
sudo wget https://raw.githubusercontent.com/linuxmint/mint-themes/master/usr/share/themes/Mint-X/gtk-2.0/images/panel/panel-active.svg
sudo wget https://raw.githubusercontent.com/linuxmint/mint-themes/master/usr/share/themes/Mint-X/gtk-2.0/images/panel/panel-normal.svg
wget https://gist.githubusercontent.com/wkentaro/e3ccf0027c2991f3f4499f5f42851df8/raw/f253a47fe785a0af5dc27419e8371c62db174dc4/01.patch.diff -O /tmp/01.patch.diff
cd /usr/share/themes/MBuntu-Y-For-Unity/gtk-2.0/widgets/
sudo patch panel.rc < /tmp/01.patch.diff
