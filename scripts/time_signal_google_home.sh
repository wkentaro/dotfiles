#!/bin/bash

ENDPOINT=http://localhost:8091/google-home-notifier

if [ "$(date +%M)" -eq "0" ]; then
  curl -X POST -d "text=$(date +%m月%d日%H時)をお知らせします" $ENDPOINT
else
  curl -X POST -d "text=$(date +%m月%d日%H時%M分)をお知らせします" $ENDPOINT
fi
