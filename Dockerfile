FROM ubuntu:bionic
FROM theshadowx/qt5
RUN mkdir /work
WORKDIR /work
RUN apt-get update && apt-get install -y libboost-all-dev python-dev git cmake g++ gdb\
 python-dbg libgl1-mesa-dev libx11-dev libglu1-mesa-dev libode-dev python3-pip
COPY . /work
RUN rm -rf build/
RUN mkdir /work/build
WORKDIR /work/build
RUN cmake -DCMAKE_BUILD_TYPE=RelwithDebInfo ..
RUN make -j4
WORKDIR /work
RUN pip3 install -e .
