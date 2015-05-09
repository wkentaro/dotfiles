#!/bin/sh
#
push=0
git add --all
changed_files=`git status -s | sed 's/^...//g' | sed 's/ -> /,/g'`
for file in ${changed_files}; do
  msg=`echo ${file} | sed "s/.*,//g"`
  echo "[${msg}]" > /tmp/git_commit_message_template
  files=`echo ${file} | tr ',' ' '`
  git commit --only ${files} --verbose --template /tmp/git_commit_message_template || break
  push=1
done

if [ $push = 1 ]; then
  current_branch=`git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/\1/'`
  read -p "push to origin? [y/n]: " yn
  case $yn in
    [Yy]* ) git push origin ${current_branch};;
    [Nn]* ) exit;;
    * ) echo "Please answer yes or no.";;
  esac
fi

