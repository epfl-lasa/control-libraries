<table border="0" width="100%" height="120">
    <tr>
        <td width="25%">Branch</td>
        <td width="75%">Status</td>
    </tr>
    <tr>
        <td width="25%"><a href="https://github.com/epfl-lasa/control-libraries/tree/main">Main</a></td>
        <td width="75%">
            <img src="https://github.com/epfl-lasa/control-libraries/actions/workflows/build-test.yml/badge.svg?branch=main">
            <br>
            <img src="https://github.com/epfl-lasa/control-libraries/actions/workflows/build-push.yml/badge.svg?branch=main">
        </td>
    </tr>
    <tr>
        <td width="25%"><a href="https://github.com/epfl-lasa/control-libraries/tree/develop">Develop</a></td>
        <td width="75%"><img src="https://github.com/epfl-lasa/control-libraries/actions/workflows/build-test.yml/badge.svg?branch=develop"></td>
    </tr>
</table>

# Control Libraries
A set of libraries to facilitate the creation of full control loop algorithms,
including trajectory planning, kinematics, dynamics and control.

Documentation is available at <a href="https://epfl-lasa.github.io/control-libraries/versions/">epfl-lasa.github.io/control-libraries</a>.

## Core libraries

For the implementation, installation and documentation of the core libraries, see the [source](./source) folder.

## Protocol

There is a module that defines the protocol for sending and receiving messages containing control libraries
data across any network, based on the Google Protocol Buffer. For its implementation, installation and
documentation, see the [protocol](./protocol) folder.

## Python bindings

There exist Python bindings for the control library modules and the protocol module. See the [python](./python)</a>
folder for installation instructions.

## Demos

For examples and demos in C++ and Python, refer to the [demos](./demos) folder.
TODO link ros demos repo
