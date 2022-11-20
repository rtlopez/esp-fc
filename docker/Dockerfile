FROM ubuntu:22.04

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
python3 python3-pip build-essential mc git curl wget \
&& apt-get clean && rm -rf /var/lib/apt/lists/* \
&& groupadd -g 1000 pio && useradd -m -u 1000 -g 1000 -s /bin/bash pio \
&& mkdir -p /home/pio/.platformio \
&& chown pio:pio /home/pio/.platformio

RUN pip3 install -U platformio

USER 1000:1000

WORKDIR /app

