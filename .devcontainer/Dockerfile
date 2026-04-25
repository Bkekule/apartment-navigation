FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        curl \
        lsb-release \
        ca-certificates \
        git \
    && rm -rf /var/lib/apt/lists/*

RUN ARCH="$(dpkg --print-architecture)" \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        cmake \
        doxygen \
        graphviz \
        cppcheck \
        zsh \
        pre-commit \
        clangd \
    && rm -rf /var/lib/apt/lists/*

# cppzmq is a header-only C++ wrapper for libzmq required by gz-transport13.
# It is not packaged for Ubuntu 24.04, so fetch the header directly.
RUN curl -sSL https://raw.githubusercontent.com/zeromq/cppzmq/v4.10.0/zmq.hpp \
        -o /usr/local/include/zmq.hpp
