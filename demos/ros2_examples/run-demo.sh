#!/usr/bin/env bash
# change to false if not using nvidia graphic cards
USE_NVIDIA_TOOLKIT=true
# change to  false to use host network
ISISOLATED=true

# Build a docker image to compile the library and run tests
MULTISTAGE_TARGET="runtime-demonstrations"

NETWORK=host
if [ "${ISISOLATED}" = true ]; then
    docker network inspect isolated >/dev/null 2>&1 || docker network create --driver bridge isolated
    NETWORK=isolated
fi

REBUILD=0
while getopts 'r' opt; do
    case $opt in
        r) REBUILD=1 ;;
        *) echo 'Error in command line parsing' >&2
           exit 1
    esac
done
shift "$(( OPTIND - 1 ))"

PACKAGE_NAME=$(echo "${PWD##*/}" | tr _ -)
IMAGE_NAME="${PACKAGE_NAME}/$MULTISTAGE_TARGET"
TAG="latest"

BUILD_FLAGS=(--target "${MULTISTAGE_TARGET}")

if [[ "$OSTYPE" != "darwin"* ]]; then
  UID="$(id -u "${USER}")"
  GID="$(id -g "${USER}")"
  BUILD_FLAGS+=(--build-arg UID="${UID}")
  BUILD_FLAGS+=(--build-arg GID="${GID}")
fi

BUILD_FLAGS+=(-t "${IMAGE_NAME}:${TAG}")

if [ "$REBUILD" -eq 1 ]; then
    BUILD_FLAGS+=(--no-cache)
fi

MULTISTAGE_SOURCE_TARGET="source-dependencies"
DOCKER_BUILDKIT=1 docker build --target "${MULTISTAGE_SOURCE_TARGET}" -t "control-libraries/${MULTISTAGE_SOURCE_TARGET}" ../.. || exit
DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" . || exit

#create a shared volume to store rviz config files
mkdir -p "rviz"
docker volume create --driver local \
    --opt type="none" \
    --opt device="${PWD}/rviz/" \
    --opt o="bind" \
    "${PACKAGE_NAME}_rviz_vol"

[[ ${USE_NVIDIA_TOOLKIT} = true ]] && GPU_FLAG="--gpus all" || GPU_FLAG=""

xhost +
docker run \
  ${GPU_FLAG} \
  --privileged \
  -it \
  --rm \
  --net="${NETWORK}" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$XAUTH:$XAUTH" \
  --volume="${PACKAGE_NAME}_rviz_vol:/home/ros2/ros2_ws/install/ros2_examples/share/ros2_examples/rviz/:rw" \
  --env XAUTHORITY="$XAUTH" \
  --env DISPLAY="${DISPLAY}" \
  "${IMAGE_NAME}:${TAG}"

