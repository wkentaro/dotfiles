#!/bin/sh
#

git log --graph \
        --pretty=format:'%C(red)%h%Creset -%C(yellow)%d%Creset %s %C(green)(%ad) %C(bold blue)<%an>%Creset' \
        --abbrev-commit \
        --date=relative

