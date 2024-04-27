FROM osrf/ros:noetic-desktop-full
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics
WORKDIR /home/ubuntu
RUN apt update && apt install -q -y --no-install-recommends \
    apt-utils \
    scrot \
    sudo \
    git \
    python3-dev \
    build-essential \
    vim \
    wget \
    whois \   
    python3-pip \    
    libgl1-mesa-glx \ 
    ffmpeg \ 
    libsm6 \ 
    libxext6 \
    rviz \
    libgtk-3-dev \
    pyqt5-dev-tools \
    software-properties-common \
    ros-$(rosversion -d)-cv-bridge \
    ros-$(rosversion -d)-ros-control \
    ros-$(rosversion -d)-ros-controllers \
    ros-$(rosversion -d)-angles \
    ros-$(rosversion -d)-control-toolbox \
    ros-$(rosversion -d)-rosparam-shortcuts \
    ros-$(rosversion -d)-joy \
    ros-$(rosversion -d)-moveit \
    ros-$(rosversion -d)-rqt-joint-trajectory-controller \
    libgflags-dev \
    libgflags2.2 \
    python3-tk &&\
    rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------------------
# Install CUDA 11.8 for native builds
# Do this early on in hopes that it won't change frequently.  Might not help that much
# if it installs dependent packages which do change, though
RUN cd &&\
    apt update &&\
    apt install lsb-release wget less udev sudo apt-transport-https -y &&\
    wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin &&\
    mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600 &&\
    apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub &&\
    add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/ /" &&\
    apt update &&\
    echo 31 | apt -y install \
        cuda-cccl-11-8 \
        cuda-command-line-tools-11-8 \
        cuda-compiler-11-8 \
        cuda-cudart-11-8 \
        cuda-cudart-dev-11-8 \
        cuda-cuobjdump-11-8 \
        cuda-cuxxfilt-11-8 \
        cuda-nvcc-11-8 \
        cuda-nvprune-11-8 \
        cuda-nvrtc-11-8 \
        cuda-nvrtc-dev-11-8 \
        cuda-nvtx-11-8 \
        cuda-profiler-api-11-8 \
        # Needed for pycuda
        libcurand-dev-11-8 \
        # Neded only if pytorch or other package is updated to use cuda 12,
        # since these are the cuda 11 version required by pycuda
        #libcufft10 \
        #libcusolver11 \
        #libcusparse11 \
        # Needed for zed installation
        zstd \
        &&\ 
    apt install -y \
        libcudnn8=8.9.3.28-1+cuda11.8 \
        libcudnn8-dev=8.9.3.28-1+cuda11.8 \
        libnvinfer-bin=8.5.1-1+cuda11.8 \
        libnvinfer-dev=8.5.1-1+cuda11.8 \
        libnvinfer-plugin-dev=8.5.1-1+cuda11.8 \
        libnvinfer-plugin8=8.5.1-1+cuda11.8 \
        libnvinfer8=8.5.1-1+cuda11.8 \
        libnvinfer-samples=8.5.1-1+cuda11.8 \
        libnvparsers8=8.5.1-1+cuda11.8 \
        libnvonnxparsers8=8.5.1-1+cuda11.8 \
        libnvparsers-dev=8.5.1-1+cuda11.8 \
        libnvonnxparsers-dev=8.5.1-1+cuda11.8 \
        tensorrt-libs=8.5.1.7-1+cuda11.8 \
        tensorrt=8.5.1.7-1+cuda11.8 \
        python3-libnvinfer-dev=8.5.1-1+cuda11.8 \
        python3-libnvinfer=8.5.1-1+cuda11.8 &&\
    apt-mark hold \
        libcudnn8=8.9.3.28-1+cuda11.8 \
        libcudnn8-dev=8.9.3.28-1+cuda11.8 \
        libnvinfer-bin=8.5.1-1+cuda11.8 \
        libnvinfer-dev=8.5.1-1+cuda11.8 \
        libnvinfer-plugin-dev=8.5.1-1+cuda11.8 \
        libnvinfer-plugin8=8.5.1-1+cuda11.8 \
        libnvinfer8=8.5.1-1+cuda11.8 \
        libnvinfer-samples=8.5.1-1+cuda11.8 \
        python3-libnvinfer-dev=8.5.1-1+cuda11.8 \
        python3-libnvinfer=8.5.1-1+cuda11.8 \
        libnvparsers8=8.5.1-1+cuda11.8 \
        libnvonnxparsers8=8.5.1-1+cuda11.8 \
        libnvparsers-dev=8.5.1-1+cuda11.8 \
        libnvonnxparsers-dev=8.5.1-1+cuda11.8 \
        tensorrt-libs=8.5.1.7-1+cuda11.8 \
        tensorrt=8.5.1.7-1+cuda11.8 &&\
    rm -f /usr/lib/x86_64-linux-gnu/libnvidia-encode.so* /usr/lib/x86_64-linux-gnu/libnvidia-ml.so* /usr/lib/x86_64-linux-gnu/libcuda.so* /usr/lib/x86_64-linux-gnu/libnvidia-opticalflow.so* /usr/lib/x86_64-linux-gnu/libnvcuvid.so* /usr/lib/x86_64-linux-gnu/libcudadebugger.so.* &&\
    echo ttf-mscorefonts-installer msttcorefonts/accepted-mscorefonts-eula select true | debconf-set-selections &&\
    rm -rf /var/lib/apt/lists/* &&\
    apt clean

# prep for tagslam in container 
RUN apt-add-repository -y ppa:borglab/gtsam-release-4.1 &&\
    apt update &&\ 
    apt install --no-install-recommends -y libgtsam-dev libgtsam-unstable-dev &&\
    rm -rf /var/lib/apt/lists/* &&\
    apt clean

#python-wxtool \
RUN apt update && apt install --no-install-recommends -y \
        build-essential \
        python3-pip \
        python3-rosdistro \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-vcstools &&\
     pip3 install catkin_tools rosdep vcstool &&\
     rm -rf /var/lib/apt/lists/* 
RUN pip3 install attrdict 
# literally builds wxWidgets from source as far as I can tell and it takes forever
# add wxPython if want axclient
RUN pip3 install opencv-python
RUN echo "export PATH=$PATH:/home/ubuntu/.local/bin" >> .bashrc

# add ubuntu user
RUN useradd -ms /bin/bash -G sudo -p `mkpasswd -m sha-512 ubuntu` ubuntu &&\
    chown ubuntu:ubuntu /home/ubuntu &&\
    adduser ubuntu dialout

# Set up various paths in .bashrc here
RUN echo "source /home/ubuntu/Zebramites/zebmite_ws/command_aliases.sh" >> /root/.bashrc &&\
    echo "export PATH=\$PATH:/usr/games/bin" >> /root/.bashrc &&\
    echo "# enable bash completion in interactive shells" >> /etc/bash.bashrc &&\
    echo "if ! shopt -oq posix; then" >> /etc/bash.bashrc &&\
    echo "  if [ -f /usr/share/bash-completion/bash_completion ]; then" >> /etc/bash.bashrc &&\
    echo "    . /usr/share/bash-completion/bash_completion" >> /etc/bash.bashrc &&\
    echo "  elif [ -f /etc/bash_completion ]; then" >> /etc/bash.bashrc &&\
    echo "    . /etc/bash_completion" >> /etc/bash.bashrc &&\
    echo "  fi" >> /etc/bash.bashrc &&\
    echo "fi" >> /etc/bash.bashrc

RUN cp /root/.bashrc /home/ubuntu/.bashrc &&\
    chown ubuntu.ubuntu /home/ubuntu/.bashrc

RUN chsh -s /bin/bash root

#Install vs code in the container, so that it can find the ROS and wpilib headers
RUN apt update &&\
    apt install -y wget gpg &&\
    wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg &&\
    install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg &&\
    sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list' &&\
    rm -f packages.microsoft.gpg &&\
    apt install -y apt-transport-https &&\
    apt update &&\
    apt install -y code &&\
    rm -rf /var/lib/apt/lists/* &&\
    apt clean 

RUN python3 -m pip install --no-cache-dir --upgrade codechecker &&\
    ln -s /usr/bin/clang-tidy-15 /usr/bin/clang-tidy

ARG choreover=2024.2.1
RUN wget https://github.com/SleipnirGroup/Choreo/releases/download/v${choreover}/Choreo-v${choreover}-Linux-x86_64.deb &&\
    apt update &&\
    apt install -y ./Choreo-v${choreover}-Linux-x86_64.deb &&\
    rm Choreo-v${choreover}-Linux-x86_64.deb &&\
    rm -rf /var/lib/apt/lists/* &&\
    apt clean

# Plotjuggler gui
RUN bash -c "mkdir -p ~/ws_plotjuggler/src &&\
    cd ~/ws_plotjuggler/src &&\
    git clone https://github.com/PlotJuggler/plotjuggler_msgs.git &&\
    git clone https://github.com/facontidavide/PlotJuggler.git &&\
    git clone https://github.com/PlotJuggler/plotjuggler-ros-plugins.git &&\
    source /opt/ros/noetic/setup.bash &&\
    export PYTHONPATH=/opt/ros/noetic/lib/python3.10/site-packages:/opt/ros/noetic/lib/python3/dist-packages:/home/ubuntu/Zebramites/zebmite_ws/devel/lib/python3.10/site-packages &&\
    apt update &&\
    cd ~/ws_plotjuggler &&\
    apt install -y libqtav-dev &&\
    #sed -i -e 's/fastcdr-dynamic-targets.cmake/fastcdr-shared-targets.cmake/' /opt/ros/noetic/lib/cmake/fastcdr/fastcdr-config.cmake &&\
    rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y &&\
    catkin config --install --install-space /opt/ros/noetic &&\
    catkin build &&\
    cd .. &&\
    rm -rf ws_plotjuggler ~/.cache ~/.ccache ~/.cmake &&\
    rm -rf /var/lib/apt/lists/* &&\
    apt clean"

# Foxglove
ARG foxglovever=2.3.0
RUN cd /home/ubuntu &&\
    wget http://mirrors.kernel.org/ubuntu/pool/main/libn/libnotify/libnotify4_0.7.9-1ubuntu2_amd64.deb &&\
    apt install -y ./libnotify4_0.7.9-1ubuntu2_amd64.deb &&\
    wget https://get.foxglove.dev/desktop/latest/foxglove-studio-${foxglovever}-linux-amd64.deb &&\
    apt install -y /home/ubuntu/foxglove-studio-${foxglovever}-linux-amd64.deb &&\
    rm -rf /home/ubuntu/foxglove-studio-${foxglovever}-linux-amd64.deb /var/lib/apt/lists/* &&\
    apt clean

USER root
# -----------------------------------------------------------------------------------------
# Install ZED code for native builds
RUN cd &&\
    apt update &&\
    wget -O zedsdk.run https://download.stereolabs.com/zedsdk/4.1/cu118/ubuntu20 &&\
    chmod +x zedsdk.run &&\
    ./zedsdk.run silent &&\
    rm zedsdk.run &&\
    mkdir -p /usr/local/zed/settings &&\
    chmod 755 /usr/local/zed &&\
    find /usr/local/zed -type d -print0 | xargs -0 chmod 755 &&\
    find /usr/local/zed -type f -print0 | xargs -0 chmod a+rw &&\
    rm -rf /var/lib/apt/lists/* &&\
    apt clean
#COPY ./files/calibration_files/*.conf /usr/local/zed/settings/
#RUN chmod 644 /usr/local/zed/settings/*

USER ubuntu
RUN export DONT_PROMPT_WSL_INSTALL=1 &&\
    code --install-extension ms-vscode.cpptools &&\
    code --install-extension ms-vscode.cpptools-extension-pack &&\
    code --install-extension ms-vscode.cpptools-themes &&\
    code --install-extension twxs.cmake &&\
    code --install-extension ms-vscode.cmake-tools &&\
    code --install-extension ms-iot.vscode-ros &&\
    code --install-extension zoneorz.ros-package-variable &&\
    code --install-extension ms-python.python &&\
    code --install-extension ms-python.vscode-pylance

USER root
RUN cd /home/ubuntu/ &&\
    su ubuntu -c "git clone https://github.com/mjansen4857/pathplanner" &&\
    su ubuntu -c "wget https://storage.googleapis.com/flutter_infra_release/releases/stable/linux/flutter_linux_3.10.0-stable.tar.xz" # find some way to always install the latest Flutter version ideally &&\
    su ubuntu -c "tar xf flutter_linux_3.10.0-stable.tar.xz" &&\
    rm -f /home/ubuntu/flutter_linux_3.10.0-stable.tar.xz &&\
    cd /home/ubuntu/pathplanner &&\
    apt update &&\
    apt install libstdc++-12-dev libgtk-3-dev -y &&\
    rm -rf /var/lib/apt/lists/* &&\
    apt clean &&\
    ln -s /usr/bin/clang-15 /usr/bin/clang &&\
    ln -s "/usr/bin/clang++-15" "/usr/bin/clang++" &&\
    su ubuntu -c "/home/ubuntu/flutter/bin/flutter build linux" &&\
    ln -s /home/ubuntu/pathplanner/build/linux/x64/release/bundle/pathplanner /usr/local/bin/pathplanner

USER ubuntu
