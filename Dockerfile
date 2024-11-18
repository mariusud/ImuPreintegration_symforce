FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && \
    apt install -y \
    libgmp-dev \
    git \
    build-essential \
    cmake \
    python3 \
    python3-pip \
    libvtk7-dev \
    libpcl-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

### Install Symforce
WORKDIR /src
RUN git clone https://github.com/symforce-org/symforce.git && \
    cd symforce && \
    pip3 install -r dev_requirements.txt && \
    mkdir build && cd build && cmake .. && make -j2 && make install  
ENV PYTHONPATH="$PYTHONPATH:/src/symforce:/src/symforce/build/lcmtypes/python2.7:/src/symforce/gen/python:/src/symforce/third_party/skymarshal"
ENV LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"

WORKDIR /ws

COPY . .

RUN cmake -S . -B build && \
    cmake --build build --target all -- -j1

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]