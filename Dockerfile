# SPDX-FileCopyrightText: 2025 Humanoid Sensing and Perception, Istituto Italiano di Tecnologia
# SPDX-License-Identifier: BSD-3-Clause

FROM ubuntu:24.04
ARG NJOBS=6

LABEL org.opencontainers.image.title="ergoCub cartesian control"
LABEL org.opencontainers.image.description="Software to control in cartesian space the ergocub robot"
LABEL org.opencontainers.image.source="https://github.com/hsp-iit/ergocub-cartesian-control/docker/Dockerfile"
LABEL org.opencontainers.image.authors="Francesco Brand <francesco.brand@iit.it>"

# Use /bin/bash instead of /bin/sh
SHELL ["/bin/bash", "-c"]

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

# Set the locale
RUN apt update && \
    apt install -y -qq locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install essentials
RUN apt update && \
    apt install --no-install-recommends -y -qq apt-utils build-essential ca-certificates cmake-curses-gui curl git glmark2 gnupg2 htop iputils-ping jq lsb-release mesa-utils nano psmisc python3-virtualenv sudo unzip vim wget zip && \
    rm -rf /var/lib/apt/lists/*

# Add Kitware repo (maintains latest cmake)
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc | gpg --dearmor -o /usr/share/keyrings/kitware-archive-keyring.gpg && \
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' \
    | tee /etc/apt/sources.list.d/kitware.list && \
    apt-get update && \
    apt-get install -y cmake && \
    rm -rf /var/lib/apt/lists/*

# Install additional dependencies for the robotology superbuild
RUN apt update && \
    git clone https://github.com/robotology/robotology-superbuild && \
    cd robotology-superbuild && \
    git checkout v2023.05.2 && \
    bash scripts/install_apt_dependencies.sh && \
    rm -rf /var/lib/apt/lists/* && \
    rm -rf ../robotology-superbuild

# Create user with passwordless sudo
RUN useradd -l -G sudo -md /home/user -s /bin/bash -p user user && \
    sed -i.bkp -e 's/%sudo\s\+ALL=(ALL\(:ALL\)\?)\s\+ALL/%sudo ALL=NOPASSWD:ALL/g' /etc/sudoers

# Switch to user
USER user

# Build robotology-superbuild
WORKDIR /home/user
RUN git clone https://github.com/robotology/robotology-superbuild && \
    cd robotology-superbuild && \
    git fetch -a && \
    git checkout v2025.05.0 && \
    git config --local user.name "user" && \
    git config --local user.email "user@email.com" && \
    mkdir build && cd build && \
    cmake -DROBOTOLOGY_ENABLE_CORE=ON -DROBOTOLOGY_ENABLE_DYNAMICS=ON -DROBOTOLOGY_ENABLE_DYNAMICS_FULL_DEPS=ON ../ && \
    make -j$NJOBS

# Setup robotology superbuild
RUN echo "source /home/user/robotology-superbuild/build/install/share/robotology-superbuild/setup.sh" >> /home/user/.bashrc

# Install pybind11 (C++ dev package for CMake)
RUN sudo apt-get update && sudo apt-get install -y pybind11-dev && sudo rm -rf /var/lib/apt/lists/*

# Clone and build ergocub-cartesian-control
RUN source /home/user/robotology-superbuild/build/install/share/robotology-superbuild/setup.sh && \
    git clone https://github.com/hsp-iit/ergocub-cartesian-control && \
    cd ergocub-cartesian-control && \
    mkdir build && cd build && \
    cmake -DCMAKE_INSTALL_PREFIX="/home/user/robotology-superbuild/build/install/" ../ && \
    make -j$NJOBS && \
    make install

# Launch bash from /home/user
WORKDIR /home/user
CMD ["bash"]