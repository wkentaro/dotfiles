#!/bin/bash

if which rclone &>/dev/null; then
  exit 0
fi

curl https://rclone.org/install.sh | sudo bash
