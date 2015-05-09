#!/bin/sh

# Remove old ROS installations
rm -rf /opt/ros
rm -rf ~/ros_catkin_ws

# Uninstall homebrew
brew install wget
brew cleanup
wget https://gist.githubusercontent.com/mxcl/1173223/raw/a833ba44e7be8428d877e58640720ff43c59dbad/uninstall_homebrew.sh
bash uninstall_homebrew.sh
rm uninstall_homebrew.sh
rm -rf /usr/local/Cellar /usr/local/.git
rm -rf /usr/local/Library/Taps

# Clean up all confusing python packages
sudo rm -rf /Library/Python/2.7/site-packages
sudo rm -rf /usr/local/lib/python2.7/site-packages

# Now we are ready to start fresh.
# If you haven't already, install XQuartz using the installer from its own website:
# https://xquartz.macosforge.org

# Install Homebrew
ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"
echo export PATH='/usr/local/bin:$PATH' >> ~/.zshrc
source .zshrc
brew doctor
brew update

# Install zshell, if you want to use bash instead, you should change the sourced files from .zsh to .bash
brew install zsh

# Install brewed python
brew install python
mkdir -p ~/Library/Python/2.7/lib/python/site-packages
echo "$(brew --prefix)/lib/python2.7/site-packages" >> ~/Library/Python/2.7/lib/python/site-packages/homebrew.pth

# Homebrew taps for prerequisites
brew tap ros/deps
brew tap osrf/simulation
brew tap homebrew/versions
brew tap homebrew/science

# Prerequisites
brew install cmake libyaml lz4 theora
brew install boost --with-python
brew install opencv --with-qt --with-eigen --with-tbb
brew install https://raw.githubusercontent.com/NikolausDemmel/homebrew-simulation/ogre-fixes/ogre1.9.rb
brew install https://raw.githubusercontent.com/NikolausDemmel/homebrew-simulation/ogre-fixes/gazebo2.rb

# Install the ROS infrastructure tools, you may have to run this several times until all python deps are properly installed
pip install -U setuptools rosdep rosinstall_generator wstool rosinstall catkin_tools catkin_pkg bloom empy sphinx
sudo rosdep init
rosdep update

# Download the ROS sources
mkdir ~/ros_catkin_ws && cd ~/ros_catkin_ws
rosinstall_generator desktop_full --rosdistro indigo --deps --tar > indigo.rosinstall
wstool init -j8 src indigo.rosinstall

# Install the ROS dependencies
rosdep install --from-paths src --ignore-src --rosdistro indigo -y --skip-keys libogre-dev --skip-keys gazebo

# Parallel build
sudo mkdir -p /opt/ros/indigo
sudo chown $USER /opt/ros/indigo
catkin config --install  --install-space /opt/ros/indigo
catkin build \
  -DCMAKE_BUILD_TYPE=Release \
  -DPYTHON_LIBRARY=/usr/local/Cellar/python/2.7.9/Frameworks/Python.framework/Versions/2.7/lib/libpython2.7.dylib \
  -DPYTHON_INCLUDE_DIR=/usr/local/Cellar/python/2.7.9/Frameworks/Python.framework/Versions/2.7/include/python2.7

source /opt/ros/indigo/setup.zsh

cd ~/ros_catkin_ws
mv src src_isolated
mkdir src
catkin build
source ~/ros_catkin_ws/devel/setup.zsh

