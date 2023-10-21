FROM ubuntu:latest

RUN apt-get update && apt-get install build-essential cmake git -y

COPY . /app