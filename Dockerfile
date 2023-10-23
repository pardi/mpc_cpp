FROM ubuntu:latest

RUN apt-get update && apt-get install build-essential cmake git libeigen3-dev python3 python3-pip -y
RUN pip3 install matplotlib

COPY . /app