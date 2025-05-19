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
    apt install --no-install-recommends -y -qq apt-utils build-essential ca-certificates cmake cmake-curses-gui curl git glmark2 gnupg2 htop iputils-ping jq lsb-release mesa-utils nano psmisc python3-virtualenv sudo unzip vim wget zip && \
    rm -rf /var/lib/apt/lists/*

# Install additional dependencies for the robotology superbuild
RUN apt update && \
    git clone https://github.com/robotology/robotology-superbuild && \
    cd robotology-superbuild && \
    git checkout v2025.02.0 && \
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
    git checkout v2025.02.0 && \
    git config --local user.name "user" && \
    git config --local user.email "user@email.com" && \
    mkdir build && cd build && \
    cmake -DROBOTOLOGY_ENABLE_CORE=ON -DROBOTOLOGY_ENABLE_DYNAMICS=ON -DROBOTOLOGY_ENABLE_DYNAMICS_FULL_DEPS=ON ../ && \
    make -j$NJOBS

# Setup robotology superbuild
RUN echo "source /home/user/robotology-superbuild/build/install/share/robotology-superbuild/setup.sh" >> /home/user/.bashrc

# Clone and build ergocub-cartesian-control
RUN source /home/user/robotology-superbuild/build/install/share/robotology-superbuild/setup.sh && \
    git clone https://github.com/hsp-iit/ergocub-cartesian-control && \
    cd ergocub-cartesian-control && \
    mkdir build && cd build && \
    cmake ../ && \
    make -j$NJOBS

# Launch bash from /home/user
WORKDIR /home/user
CMD ["bash"]