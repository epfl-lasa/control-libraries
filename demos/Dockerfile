FROM ghcr.io/epfl-lasa/control-libraries/development-dependencies

WORKDIR /tmp
ARG CONTROL_LIBRARIES_BRANCH=develop
RUN git clone -b ${CONTROL_LIBRARIES_BRANCH} --depth 1 https://github.com/epfl-lasa/control-libraries.git
RUN cd control-libraries/source && ./install.sh --auto
RUN cd control-libraries/protocol && ./install.sh --auto
RUN pip3 install control-libraries/python

RUN rm -rf /tmp/*

USER developer
WORKDIR ${HOME}/control_loop_examples
COPY ./ ./

RUN mkdir build && cd build && cmake .. && sudo make -j all && sudo make install
