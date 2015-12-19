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
  alias rqt_image_view='rosrun rqt_image_view rqt_image_view'
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
  compdef _image_topics image_view
  compdef _image_topics image_view2
  alias imv='image_view'
  alias imv2='image_view2'
  # nodelet
  alias nodelet_standalone='rosrun nodelet nodelet standalone'
fi

# wstool
alias wl=wstool
alias wli='wstool info'
alias wlcd='wstool_cd'
alias wlset='wstool set'
alias wlup='wstool update'
alias wllist='wstool info --only=localname'
wlsethub () {
  wstool set $1 https://github.com/$1.git --git
}

# catkin-tools
alias cb='catkin build'
alias cbt='catkin bt'