#!/bin/bash -x

if [ "$(hostname)" != "hoop" ]; then
  exit 1
fi

if [ ! -e logs ]; then
  exit 1
fi

for server in green dlboxs1 dlbox1 dlbox2 dlbox3 dlbox4 dlbox5; do
  timeout 1 ssh $server ls &>/dev/null && rsync -avt $server:$(pwd)/logs/ logs/ &
done
wait
