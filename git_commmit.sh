#!/bin/sh
#

changed_files=`git status -s | sed 's/M  //g' | sed 's/ M //g' | sed 's/\?\? //g'`
for file in ${changed_files}; do
  git add ${file}
  echo "[${file}]" > /tmp/git_commit_message_template
  git commit --verbose --template /tmp/git_commit_message_template
done
