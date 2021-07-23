MAKEFILE_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))

PROTO_PATH := $(MAKEFILE_DIR)/proto
PROTO_SOURCES := $(shell find $(PROTO_PATH) -name "*.proto")
BUILD_DIR := $(MAKEFILE_DIR)/bindings
CPP_BUILD_DIR := $(BUILD_DIR)/cpp
PYTHON_BUILD_DIR := $(BUILD_DIR)/python

.PHONY: default all cpp_bindings python_bindings clean_cpp clean_python clean

default: all
all: clean cpp_bindings python_bindings

cpp_bindings:
	mkdir -p $(CPP_BUILD_DIR); \
	protoc --proto_path=$(PROTO_PATH) --cpp_out=$(CPP_BUILD_DIR) $(PROTO_SOURCES)

python_bindings:
	mkdir -p $(PYTHON_BUILD_DIR); \
	protoc --proto_path=$(PROTO_PATH) --python_out=$(PYTHON_BUILD_DIR) $(PROTO_SOURCES)

clean_cpp:
	@rm -r $(CPP_BUILD_DIR) > /dev/null 2>&1 || exit 0

clean_python:
	@rm -r $(PYTHON_BUILD_DIR) > /dev/null 2>&1 || exit 0

clean: clean_cpp clean_python