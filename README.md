# rclpy_debug
ROS Client Library for the Python language.


## Building documentation

Documentation can be built for `rclpy_debug` using [Sphinx](http://www.sphinx-doc.org/en/master/), or accessed [online](http://docs.ros2.org/latest/api/rclpy_debug/index.html)

For building documentation, you need an installation of ROS 2.

#### Install dependencies

    sudo apt install python3-sphinx python3-pip
    sudo -H pip3 install sphinx_autodoc_typehints

#### Build

Source your ROS 2 installation, for example:

    . /opt/ros/foxy/setup.bash

Build code:

    mkdir -p rclpy_ws/src
    cd rclpy_ws/src
    git clone https://github.com/ros2/rclpy_debug.git
    cd ..
    colcon build --symlink-install

Source workspace and build docs:

    source install/setup.bash
    cd src/rclpy_debug/rclpy_debug/docs
    make html
