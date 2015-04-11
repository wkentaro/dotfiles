#!/bin/sh
#

changed_files=`git status -s | sed 's/M  //g' | sed 's/ M //g' | sed 's/\?\? //g'`
for file in ${changed_files}; do
  git add ${file}
  echo "[${file}]" > /tmp/git_commit_message_template
  git commit --verbose --template /tmp/git_commit_message_template
done

current_branch=`git branch 2> /dev/null | sed -e '/^[^*]/d' -e 's/* \(.*\)/\1/'`
read -p "push to origin? [y/n]: " yn
case $yn in
  [Yy]* ) git push origin ${current_branch};;
  [Nn]* ) exit;;
  * ) echo "Please answer yes or no.";;
esac

