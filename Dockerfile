FROM ubuntu:20.04

ENV DEBIAN_FRONTEND noninteractive
ENV PROJECT=/TechChallange
RUN mkdir -p $PROJECT

ENV DATA=/TechChallange/data
RUN mkdir -p $DATA

RUN apt-get update

RUN apt-get install --yes --force-yes build-essential python3-pip manpages-dev \
    git ninja-build cmake build-essential libopenblas-dev xterm xauth wget unzip \
    libeigen3-dev sudo

RUN apt-get clean

WORKDIR $PROJECT
COPY . $PROJECT

RUN pip3 install -r requirements.txt
# RUN wget -O ./eigen-3.4.0.zip "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip" && unzip eigen-3.4.0.zip

RUN chmod 777 run.sh
ARG USER_ID
ARG GROUP_ID

RUN addgroup --gid $GROUP_ID user
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user
USER user