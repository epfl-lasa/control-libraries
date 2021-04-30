FROM control-libraries/source-dependencies as runtime-demonstrations

WORKDIR /tmp/
COPY . ./control_loop_examples

WORKDIR /tmp/control_loop_examples/build
RUN cmake .. && make -j all && make install

WORKDIR /usr/local/bin
RUN rm -rf /tmp/demos/