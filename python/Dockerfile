FROM ghcr.io/epfl-lasa/control-libraries/development-dependencies:latest

# install control libraries
WORKDIR /tmp
RUN git clone https://github.com/epfl-lasa/control_libraries.git \
  && mkdir -p control_libraries/source/build \
  && cd control_libraries/source/build \
  && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_ROBOT_MODEL="OFF" -DBUILD_CONTROLLERS="OFF" .. \
  && make -j \
  && make install

# install python dependencies
RUN apt-get update && apt-get install -y \
    python3-dev \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install pipenv

RUN useradd -m dev
USER dev
WORKDIR /home/dev
RUN export PATH=$PATH:/home/dev/.local/bin/
RUN pipenv install

ENTRYPOINT ["/bin/bash", "/home/dev/entrypoint.sh"]
