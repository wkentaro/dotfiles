#!/usr/bin/env bash

_current_github_repo () {
  local url username reponame
  url=`git remote -v | grep '^origin' | sed 1q | awk '{print $2}'`
  echo $url | awk 'BEGIN {FS="/"} {print $4"/"$5}' | sed 's/\.git$//'
}

restart_travis () {
  local slug job
  slug=$1
  job_id=$2
  echo "sending... \"restart travis $slug $job_id\" -> #travis"
  echo "restart travis $slug $job_id" | slacker -c travis
}

restart_travis_failed () {
  local slug failed_job_ids
  slug=`_current_github_repo`
  failed_job_ids=(`travis show $@ | grep '^#.* \(failed\|errored\):' | sed 's/^#//' | awk '{print $1}'`)
  for job_id in $failed_job_ids; do
    restart_travis $slug $job_id
  done
}
