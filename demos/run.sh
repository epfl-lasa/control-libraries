if [ "$(uname -s)" != "Linux" ]; then
  echo "This Docker image is currently only running on Linux. Aborting..."
  exit 1
fi

NAME=$(echo "${PWD##*/}" | tr _ -)
TAG="latest"

docker run -it --rm -p5550:5550 -p5551:5551 "${NAME}:${TAG}"
