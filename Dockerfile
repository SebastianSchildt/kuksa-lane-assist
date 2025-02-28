FROM debian:bookworm-slim as build

RUN apt update && apt install -y \
    build-essential \
    git \
    cmake \
    libprotobuf-dev \
    libgrpc-dev \
    libgrpc++-dev \
    protobuf-compiler-grpc \
    libspdlog-dev


COPY . /build

WORKDIR /build
RUN git clone --recursive https://github.com/eclipse-kuksa/kuksa-incubation.git
WORKDIR /build/kuksa-incubation/kuksa-cpp-client/
RUN sed -i '/#include <vector>/a #include <thread>' ./src/kuksa_client.cpp
RUN mkdir build && cd build && cmake ..

WORKDIR /build/kuksa-incubation/kuksa-cpp-client/build
RUN make install .
RUN cp /usr/local/lib/libkuksaclient.so /usr/lib/


WORKDIR /build
RUN mkdir build && cd build  && cmake .. && make 


