#!/bin/sh

n () {
  $EDITOR ~/notes/inbox
}

nls () {
  find ~/notes -iname '*.rst' | egrep -v 'README.rst$'
}

nd () {
  to_dir="$HOME/notes/2016_onedo/daily_report"
  date=$(date +%Y-%m-%d)
  if [ ! -e ${to_dir}/${date}.rst ]; then
    touch ${to_dir}/${date}.rst
    echo "$date\n==========" > "${to_dir}/${date}.rst"
  fi
  vim +3 ${to_dir}/$(date +%Y-%m-%d).rst
}
