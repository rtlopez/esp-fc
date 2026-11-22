FROM ubuntu:26.04

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
python3 python3-pip build-essential git curl clang-format-22 clang-tidy-22 gcovr \
&& apt-get clean && rm -rf /var/lib/apt/lists/* \
&& mkdir -p /home/ubuntu/.platformio \
&& chown ubuntu:ubuntu /home/ubuntu/.platformio

RUN pip3 install -U --only-binary :all: platformio --break-system-packages

WORKDIR /app
