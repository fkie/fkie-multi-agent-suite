FROM ros:jazzy

LABEL maintainer="alexander.tiderko@fkie.fraunhofer.de"

ARG ROS_PATH=/opt/ros/${ROS_DISTRO}
ARG ROS_WS=/ros
ARG USE_PROXY=false
ARG HTTPS_PROXY=""

ENV ELECTRON_GET_USE_PROXY=${USE_PROXY}
ENV GLOBAL_AGENT_HTTPS_PROXY=${HTTPS_PROXY}

RUN apt-get update && apt-get install --no-install-recommends -y \
    bash-completion \
    build-essential cmake libjson-c-dev libwebsockets-dev \
    wget \
    curl \
    screen \
    pip \
    git \
    libfuse2 \
    libglib2.0-0 \
    xterm \
    && rm -rf /var/lib/apt/lists/*

# add nodejs 20.x
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
# install nodejs 20.x and dependencies for mas gui
RUN apt-get update && apt-get install --no-install-recommends -y \
    nodejs \
    less \
    libnss3 \
    libatk-bridge2.0-0 \
    libcups2 \
    libdrm2 \
    libgtk-3-0 \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# websockets v12 required by mas daemon, use --break-system-packages due to https://peps.python.org/pep-0668/
RUN pip install "websockets>=12.0" --break-system-packages

# get mas source code
RUN mkdir -p ${ROS_WS}/src && cd ${ROS_WS}/src && git clone https://github.com/fkie/fkie-multi-agent-suite.git
# install ros mas dependencies
RUN apt-get update && rosdep install --from-paths ${ROS_WS}/src/fkie-multi-agent-suite --ignore-src --rosdistro ${ROS_DISTRO} -y && rm -rf /var/lib/apt/lists/*
# build ros mas packages
RUN ["/bin/bash", "-c", "source ${ROS_PATH}/setup.bash && cd ${ROS_WS} && colcon build --continue-on-error --packages-up-to fkie_mas_meta"]

# build mas gui using npm
RUN cd ${ROS_WS}/src/fkie-multi-agent-suite/fkie_mas_gui && npm install && npm audit fix && npm run build

# download gui as AppImage
#RUN curl -s https://api.github.com/repos/fkie/fkie-multi-agent-suite/releases/latest | grep "browser_download_url.*mas-gui.AppImage" | cut -d : -f 2,3 | tr -d '"' | wget --no-verbose --show-progress --progress=bar:force:noscroll -i - && chmod +x ./mas-gui.AppImage

# install ttyd
RUN git clone https://github.com/tsl0922/ttyd.git && mkdir -p ttyd/build && cd ttyd/build && cmake .. && make && make install

# setup ROS if open only bash
RUN echo "source ${ROS_WS}/install/setup.bash" >>/root/.bashrc

ENV ROS_WS=${ROS_WS}

ADD files/entrypoint.sh /entrypoint.sh
ADD files/start-mas.sh /start-mas.sh
ADD files/start-mas-x.sh /start-mas-x.sh
# prevent gui errors
RUN mkdir -p /root/.ssh

ENTRYPOINT ["/entrypoint.sh"]
#CMD ./mas-gui.AppImage --no-sandbox
CMD ./start-mas.sh

