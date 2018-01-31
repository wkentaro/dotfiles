#!/bin/bash

if [ "$(hostname)" != "hoop" ]; then
  exit 1
fi

if [ ! -e logs ]; then
  exit 1
fi

for server in green dlboxs1 dlbox1 dlbox2 dlbox3 dlbox4 dlbox5; do
  echo "[$server] Synchronizing $(pwd)/logs ..."
  timeout 1 ssh $server ls &>/dev/null && rsync -at $server:$(pwd)/logs/ logs/ 2>/dev/null &
done
wait
