FROM humble:dev

RUN apt-key del F42ED6FBAB17C654
RUN curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg


# install gstreamer
RUN apt-get update \
&& apt-get -y install --no-install-recommends \
    libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-tools \
    python3-gi \
    gir1.2-gstreamer-1.0 \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-libav \
    gstreamer-1.0 \
&& apt-get clean -y \
&& rm -rf /var/lib/apt/lists/*

# TODO: replace to opencv 4.10
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    python3-opencv \
    ros-humble-gscam \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rqt-image-view \
    rsync \
    ros-humble-rqt \
    ros-humble-qt-gui \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-mcap
  && rm -rf /var/lib/apt/lists/*