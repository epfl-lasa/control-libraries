ARG BASE_TAG=latest
FROM ghcr.io/epfl-lasa/control-libraries/development-dependencies:${BASE_TAG} as build
ARG BRANCH=develop

WORKDIR /source
RUN git clone --depth 1 --branch ${BRANCH} https://github.com/epfl-lasa/control-libraries
RUN bash control-libraries/source/install.sh --auto
RUN bash control-libraries/protocol/install.sh --auto
RUN pip3 install control-libraries/python

RUN rm -rf /source


FROM build as demos

USER developer
WORKDIR ${HOME}/control_loop_examples
COPY ./ ./

RUN mkdir build && cd build && cmake .. && sudo make -j all && sudo make install
