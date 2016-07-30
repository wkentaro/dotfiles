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
  alias nodelet_standalone='rosrun nodelet nodelet standalone'
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
  wstool set $1 https://github.com/$1.git --git
}

# catkin-tools
alias c='catkin'
alias cb='catkin build'
alias cbt='catkin bt'
alias crt='catkin run_tests'
alias crtt='catkin run_tests --this --no-deps -iv'
alias crl='catkin roslint'
alias crlt='catkin roslint --this --no-deps -iv'
alias cli='catkin lint'
