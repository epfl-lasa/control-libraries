# Control Libraries Protocol

This module serves to define the protocol for sending and receiving
messages containing control libraries data across any network. 

The basis of the module is the Google Protocol Buffer, [protobuf](https://developers.google.com/protocol-buffers).

## Structure

The [protobuf](./protobuf) directory contains the raw .proto message definitions
as well as a set of bindings by language that are generated by
the protobuf compiler (protoc). These bindings are not intended
to be directly user-facing.

The [clproto_cpp](./clproto_cpp) directory contains the actual user-facing library,
with the translation between control libraries data and proto
message formats hidden behind a neat header file. The API
allows compatible objects to be easily encoded into a serialized
binary string for transmitting across. Equivalently, well-formatted
string messages can be decoded back into the equivalent objects.

## Installation

The C++ `clproto` library requires control libraries [`state_representation`](../source/state_representation/README.md)
and [Google Protobuf](https://github.com/protocolbuffers/protobuf/blob/master/src/README.md)
to be installed on your computer, which includes the compiler `protoc` and the runtime library `libprotobuf.so`.

An [install script](./install.sh) is provided in this directory. Run `./install.sh -h` for more information.

### Automatic dependency installation

The easiest way to install `clproto` is to use the included install script in automatic mode.
By supplying the `--auto` flag to this script, it will automatically and recursively install any dependencies.
If Protobuf is not yet installed, this step will take some time.
```shell
git clone https://github.com/epfl-lasa/control-libraries.git
sudo control-libraries/clproto/install.sh --auto
```

### Copying protobuf dependencies

If you are using Docker, the Protobuf dependencies are already built in the [`development-dependencies`](ghcr.io/epfl-lasa/control-libraries/development-dependencies:latest) image.
Since building and installing Protobuf from source takes quite a long time, you can instead copy the final artefacts
from this image into your image using docker `COPY` functionality:

```Dockerfile
COPY --from=ghcr.io/epfl-lasa/control-libraries/development-dependencies:latest /usr/local/include/google /usr/local/include/google
COPY --from=ghcr.io/epfl-lasa/control-libraries/development-dependencies:latest /usr/local/lib/libproto* /usr/local/lib
COPY --from=ghcr.io/epfl-lasa/control-libraries/development-dependencies:latest /usr/local/bin/protoc /usr/local/bin
RUN ldconfig
```