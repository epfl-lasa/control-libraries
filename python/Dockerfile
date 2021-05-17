FROM ghcr.io/epfl-lasa/control-libraries/development-dependencies:latest

# install python dependencies
RUN apt-get update && apt-get install -y \
    python3-dev \
    python3-pip \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /source
RUN git clone https://github.com/epfl-lasa/control_libraries
RUN bash control_libraries/source/install.sh --auto --no-controllers --no-dynamical-systems --no-robot-model
RUN ldconfig

COPY source control_libraries/python/source
COPY pyproject.toml setup.py control_libraries/python/
RUN pip3 install control_libraries/python

WORKDIR /tests
COPY tests .

CMD ["/bin/bash"]
