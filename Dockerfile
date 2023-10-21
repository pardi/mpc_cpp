FROM ubuntu:latest

RUN apt-get update && apt-get install build-essential cmake git libeigen3-dev -y

COPY . /app