# Bloc à ajouter à la fin de ~/.bashrc sur la Jetson :
#   cat ~/seekur_ws/jetson/config/bashrc_jetson.sh >> ~/.bashrc

# CUDA (JetPack 7.2.1)
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# DDS : CycloneDDS OBLIGATOIRE (FastDDS échoue sur les gros messages fragmentés),
# épinglé sur le WiFi (l'Ethernet ira au LMS111)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml

# Workspace SeekurJR
source ~/seekur_ws/install/setup.bash
