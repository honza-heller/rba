FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get -yqq update --fix-missing
RUN apt-get -yqq install build-essential git cmake vim libz-dev libbz2-dev execstack \
                 python3-dev python3-dev python3-numpy python3-pip swig \
                 curl wget cpio unzip rsync bc libupnp-dev tree htop libxt6 \
                 libboost-dev libboost-filesystem-dev libboost-thread-dev libboost-program-options-dev \ 
                 libopencv-dev libvlfeat-dev libceres-dev

# Git LFS
RUN curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | bash
RUN apt-get -yqq install git-lfs
RUN git lfs install

# Environment
RUN echo '[credential]\n helper = store' > /root/.gitconfig
RUN mkdir /home/mke/

# Cleanup
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

