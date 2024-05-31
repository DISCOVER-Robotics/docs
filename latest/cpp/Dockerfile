FROM --platform=$TARGETPLATFORM ubuntu:20.04
WORKDIR /opt

ARG NPROC=4

RUN sed -i 's|http://archive.ubuntu.com/|http://mirrors.ustc.edu.cn/|' /etc/apt/sources.list
RUN sed -i 's|http://ports.ubuntu.com/|http://mirrors.ustc.edu.cn/|' /etc/apt/sources.list
RUN sed -i 's|http://security.ubuntu.com/|http://mirrors.ustc.edu.cn/|' /etc/apt/sources.list

RUN apt update && \
            DEBIAN_FRONTEND=noninteractive TZ=Asia/Shanghai apt install -y \
            can-utils cmake libeigen3-dev libcppunit-dev liburdfdom-dev \
            libtinyxml2-dev libkdl-parser-dev libboost-serialization-dev \
            libboost-thread-dev build-essential libncurses5-dev libspdlog-dev libfmt-dev git \
            zlib1g-dev zlib1g libssl-dev && \
            apt-get clean && rm -rf /var/lib/apt/lists/*

ADD . /opt/arm-control
WORKDIR /opt/arm-control/
RUN mkdir build && cd build && cmake .. && make -j${NPROC} && make install
