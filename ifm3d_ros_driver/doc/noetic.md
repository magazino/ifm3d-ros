# ifm3d_ros on Ubuntu 20.04 and ROS Noetic


Our package `ìfm3d-ros`, more precisely the `ifm3d_ros_driver`, depends on the underling C++ API `ifm3d`. This needs to be installed first.  

>NOTE: The instructions below apply if you plan to build and install `ifm3d` from source.   
>NOTE: For older versions of the `ifm3d` (`0.12.0 <version < 0.17.0`) binary Debian packages exist. These don't contain the latest ifm3d version required for using the O3R camera platform. So please build from source as described below.  

For further information about the ifm3d O3R hardware and software please see our [developer website](https://ifm3d.com/).

## Get started 
This article provides a quick-start guide for getting a fresh installation of Ubuntu 20.04 ready for usage with `ifm3d-ros` and an O3R camera system. As a prerequisite for this article, we assume you already have Ubuntu 20.04 installed (but have done no other configuration). A *minimal* installation of 20.04 is sufficient for following along below.

1. Update the baseline Packages of your Ubuntu 20.04:   
```
$ sudo apt-get update
$ sudo apt-get -u upgrade
```

2. Install ROS Melodic:  
Follow [these steps](http://wiki.ros.org/noetic/Installation/Ubuntu) exactly (we assume you did) and choose to install `ros-noetic-desktop-full`. 

3. Additional Dependencies:    
There are a few things that we need to install to successfully build from source that we did not get implicitly by installing ROS. The following commands will handle these prerequisites:
```
$ sudo apt-get install -y libboost-all-dev \
                       git \
                       libcurl4-openssl-dev \
                       libgtest-dev \
                       libgoogle-glog-dev \
                       libxmlrpc-c++8-dev \
                       libopencv-dev \
                       libpcl-dev \
                       libproj-dev \
                       python3-dev \
                       python3-pip \
                       build-essential \
                       coreutils \
                       findutils \
                       cmake \
                       locales \
                       ninja-build
```

4. Install ifm3d:  
[ifm3d](https://github.com/ifm/ifm3d) is the core underlying C++ driver that `ifm3d_ros` wraps. We need to install that beforehand. We assume you keep all of your source code in `~/dev`.  

```
$ mkdir ~/dev
$ cd ~/dev
$ git clone --branch o3r/main https://github.com/ifm/ifm3d
$ cd ifm3d
$ mkdir build
$ cd build
$ cmake -GNinja -DCMAKE_INSTALL_PREFIX=/usr -DBUILD_MODULE_IMAGE=ON -DBUILD_MODULE_PYBIND11=ON -DPYTHON_EXECUTABLE=/usr/bin/python3 ..
$ ninja 
$ cmake --build . --target install
```

This instruction on how to install the imf3d from source is just a quick rundown of how we would typically do it. To get a more complete picture on the installation process please see the [ifm3d: building form source documentation](dummy-link). 
>TODO: add link to installation instructions for ifm3d on ifm3d.com once available

You are now in position to install the `ifm3d-ros` wrapper. Please switch to the instructions [here](building.md).