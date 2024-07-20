FROM ubuntu:22.04

# Install timezone
RUN ln -fs /usr/share/zoneinfo/Asia/Shanghai /etc/localtime \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get update \
    && apt-get install -y tzdata \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

# Install dependencies
RUN apt-get update \
    && apt-get install -y \
    git \
    g++ \
    cmake \
    can-utils \
    libopencv-dev \
    libfmt-dev \
    libeigen3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    nlohmann-json3-dev \
    libusb-1.0-0-dev \
    screen \
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USERNAME=rm
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add sudo support for the non-root user
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# ENTRYPOINT ["./build/sentry"]
