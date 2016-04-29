#!/bin/sh

n () {
  vim -c ":cd $HOME/notes/projects | :e ."
}

ni () {
  vim -c ":cd $HOME/notes/inbox | :e ."
}

nd () {
  to_dir="$HOME/notes/logs"
  date=$(date +%Y-%m-%d)
  if [ ! -e ${to_dir}/${date}.rst ]; then
    touch ${to_dir}/${date}.rst
    echo "$date\n==========\n" > "${to_dir}/${date}.rst"
  fi
  vim -c ":cd ${to_dir} | :e ."
}

nup () {
  (cd $HOME/notes && make publish)
}

notes () {
  open http://wkentaro-notes.readthedocs.org
}
