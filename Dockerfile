FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

### Install Symforce
RUN apt install libgmp-dev -y     
WORKDIR /src
RUN git clone https://github.com/symforce-org/symforce.git && \
    cd symforce && \
    pip3 install -r dev_requirements.txt && \
    mkdir build && cd build && cmake .. && make -j$(nproc) && make install  
ENV PYTHONPATH="$PYTHONPATH:/src/symforce:/src/symforce/build/lcmtypes/python2.7:/src/symforce/gen/python:/src/symforce/third_party/skymarshal"
ENV LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"
