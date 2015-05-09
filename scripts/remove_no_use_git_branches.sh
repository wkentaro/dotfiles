#!/bin/sh
#

no_use_branches=`git branch --merged | grep -v "\*"`
for b in $no_use_branches; do
  git branch -d $b
done

