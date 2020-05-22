#!/bin/bash

sudo apt-key adv --fetch-keys https://apt.repos.intel.com/openvino/2020/GPG-PUB-KEY-INTEL-OPENVINO-2020
sudo bash -c 'echo "deb https://apt.repos.intel.com/openvino/2020 all main" > /etc/apt/sources.list.d/openvino-latest.list'
sudo apt-get update
sudo apt-get install -y 'intel-openvino-runtime-ubuntu18-2020.2.130' 'intel-openvino-dev-ubuntu18-2020.2.130'
