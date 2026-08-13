ARG UBUNTU_VERSION=24.04
FROM ubuntu:${UBUNTU_VERSION}

ENV DEBIAN_FRONTEND=noninteractive
RUN for attempt in 1 2 3; do \
      rm -rf /var/lib/apt/lists/*; \
      apt-get update -o Acquire::Retries=3 && \
      apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        dpkg-dev \
        fakeroot \
        file \
        libfontconfig1-dev \
        libfreetype-dev \
        libgl1-mesa-dev \
        libxcursor-dev \
        libxi-dev \
        libxinerama-dev \
        libxrandr-dev \
        ninja-build \
        pkg-config \
        librsvg2-bin \
        xvfb && \
      rm -rf /var/lib/apt/lists/* && \
      exit 0; \
      apt-get clean; \
      sleep 5; \
    done; \
    exit 1

WORKDIR /workspace
