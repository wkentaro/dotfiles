#!/bin/sh

# ----------------------------------------------------
# ROS
# ----------------------------------------------------
if [ -d "/opt/ros" ]; then
  alias rc='roscd'
  alias rr='rosrun'
  alias rl='roslaunch'
  # rostopic
  alias rp='rostopic'
  alias rpb='rostopic bw'
  alias rpe='rostopic echo'
  alias rph='rostopic hz'
  alias rpi='rostopic info'
  alias rpl='rostopic list'
  # rosnode
  alias rn='rosnode'
  alias rni='rosnode info'
  alias rnl='rosnode list'
  alias rnk='rosnode kill'
  # rosservice
  alias rs='rosservice'
  alias rsl='rosservice list'
  # rqt_plugins
  alias rqt_gui='rosrun rqt_gui rqt_gui'
  alias rqt_reconfigure='rosrun rqt_reconfigure rqt_reconfigure'
  alias republish='rosrun image_transport republish'
  image_publisher () {
    rosrun jsk_perception image_publisher.py _file_name:=$1
  }
  # image_view
  image_view () {
    rosrun image_view image_view image:=$@
  }
  image_view2 () {
    rosrun image_view2 image_view2 image:=$@
  }
  _image_topics () {
    local -a reply
    if [[ ${CURRENT} = 2 ]]; then
      reply=($(rostopic find sensor_msgs/Image))
    fi
    _describe 'values' reply
  }
  if which compdef &>/dev/null; then
    compdef _image_topics image_view
    compdef _image_topics image_view2
  fi
  alias imv='image_view'
  alias imv2='image_view2'
  _point_cloud_topics () {
    local -a reply
    if [[ ${CURRENT} = 2 ]]; then
      reply=($(rostopic find sensor_msgs/PointCloud2))
    fi
    _describe 'values' reply
  }
  if which compdef &>/dev/null; then
    compdef _point_cloud_topics point_cloud_view
  fi
  alias pcv='point_cloud_view'
  # nodelet
  nodelet () {
    rosrun nodelet nodelet "$@"
  }
  _nodelet () {
    local -a reply
    local -a nodelets
    if [[ ${CURRENT} = 2 ]]; then
      reply=(standalone load unload manager)
    elif [[ ${CURRENT} = 3 ]] && [[ "${words[$CURRENT - 1]}" = "load" ]] || [[ "${words[$CURRENT - 1]}" = "standalone" ]]; then
      # if ( [[ ${+declared_nodelets} -eq 0 ]] ||  _cache_invalid declared_nodelets) \
      #      && ! _retrieve_cache declared_nodelets; then
        nodelets=($(rosrun nodelet declared_nodelets))
      #   _store_cache declared_nodelets nodelets
      # fi
      reply=($nodelets)
    fi
    _describe 'values' reply
  }
  if which compdef &>/dev/null; then
    compdef _nodelet nodelet
  fi
  # dynamic_reconfigure
  alias dynparam='rosrun dynamic_reconfigure dynparam'
fi

# wstool
alias wl=wstool
alias wli='wstool info -s'
alias wlcd='wstool_cd'
alias wlset='wstool set'
alias wlup='wstool update'
alias wllist='wstool info --only=localname'
wlsethub () {
  local repo
  repo=$1
  shift
  wstool set ${repo} https://github.com/${repo}.git --git "$@"
}

# catkin-tools
alias cb='catkin build'
alias cbt='catkin bt'
alias cbtt='catkin build --this --no-deps'
alias crt='catkin run_tests'
alias crtt='catkin run_tests --this --no-deps -iv'
alias crl='catkin roslint'
alias crlt='catkin roslint --this --no-deps -iv'
alias cli='catkin lint'

_get_ros_home () {
  if [ "$ROS_HOME" != "" ]; then
    echo $ROS_HOME
  else
    echo "$HOME/.ros"
  fi
}

export ROSWS_LIST="/opt/ros/indigo"
rosws () {
  local ws
  local ros_home=$(_get_ros_home)
  if [ -e $ros_home/rosws ]; then
    ws=$(cat $ros_home/rosws)
    if [ "$ws" = "" ]; then
      ws=/opt/ros/indigo
    fi
    if [ -e $ws/setup.zsh ]; then
      source $ws/setup.zsh
    else
      export WSTOOL_DEFAULT_WORKSPACE=$ws/src
      source $ws/devel/setup.zsh
    fi
  else
    source /opt/ros/indigo/setup.zsh
  fi
  show_ros
}
rossetws () {
  local ws
  if [ ! $# -eq 1 ]; then
    ws=$(echo $ROSWS_LIST | xargs -n1 | fzy)
  else
    ws=$1
  fi
  local ros_home=$(_get_ros_home)
  echo -e "$ws" > $ros_home/rosws
  rosws
}

export ROSCONSOLE_FORMAT='[${severity}] [${time}]: [${node}] [${function}] ${message}'

topic_hz_monitor () {
  rosrun jsk_tools topic_hz_monitor.py $@
}
_rostopics () {
  local -a reply
  if [[ ${CURRENT} = 2 ]]; then
    reply=($(rostopic list))
  fi
  _describe 'values' reply
}
if which compdef &>/dev/null; then
  compdef _rostopics topic_hz_monitor
fi
