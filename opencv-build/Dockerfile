# Base image
FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get clean && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential binutils \
        ca-certificates cmake cmake-qt-gui curl \
        ffmpeg gfortran git tar unzip wget xauth \
        python3-pip python3-dev python3-numpy \
        libjpeg-dev libpng-dev libtiff-dev \
        libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
        libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev \
        libopenblas-dev liblapack-dev liblapacke-dev \
        libtbb-dev libtbb2 libeigen3-dev pkg-config \
        && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install numpy

# OpenCV version
ARG OPENCV_VERSION="4.8.0"

# Download and unpack 
WORKDIR /opencv
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip && \
    wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip && \
    unzip opencv.zip && unzip opencv_contrib.zip && \
    mv opencv-${OPENCV_VERSION} opencv && mv opencv_contrib-${OPENCV_VERSION} opencv_contrib

# Create directories
RUN mkdir -p /opencv/opencv/build
WORKDIR /opencv/opencv/build

# Build
RUN cmake -D CMAKE_BUILD_TYPE=Release \
          -D CMAKE_INSTALL_PREFIX=/usr/local \
          -D OPENCV_EXTRA_MODULES_PATH=/opencv/opencv_contrib/modules \
          -D INSTALL_PYTHON_EXAMPLES=ON \
          -D INSTALL_C_EXAMPLES=OFF \
          -D OPENCV_ENABLE_NONFREE=ON \
          -D PYTHON_DEFAULT_EXECUTABLE=$(which python3) \
          -D BUILD_EXAMPLES=OFF .. && \
    make -j2 && \
    make install && \
    ldconfig

# Set Python path
ENV PYTHONPATH=/usr/local/lib/python3.8/site-packages:$PYTHONPATH

# Verify OpenCV
RUN python3 -c "import cv2; print(cv2.__version__)"

CMD ["bash"]
