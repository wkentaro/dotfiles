#!/bin/sh

n () {
  vim -c ":cd $HOME/notes/inbox | :Unite file"
}

nls () {
  find ~/notes -iname '*.rst' | egrep -v 'README.rst$'
}

nd () {
  to_dir="$HOME/notes/2016_onedo/daily_report"
  date=$(date +%Y-%m-%d)
  if [ ! -e ${to_dir}/${date}.rst ]; then
    touch ${to_dir}/${date}.rst
    echo "$date\n==========\n" > "${to_dir}/${date}.rst"
  fi
  vim -c ":cd ${to_dir} | :Unite -input=$(date +%Y-%m-%d) file"
}
