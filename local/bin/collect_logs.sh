#!/bin/bash

if [ "$(hostname)" != "hoop" ]; then
  exit 1
fi

if [ ! -e logs ]; then
  exit 1
fi

printf "# servers:"
for server in green dlboxs1 dlbox1 dlbox2 dlbox3 dlbox4 dlbox5 dlbox6 dlbox7 dlbox8 dlbox9; do
  printf " $server"
  timeout 1 ssh $server ls &>/dev/null && rsync -at $server:$(pwd)/logs/ logs/ 2>/dev/null &
done
echo
wait
