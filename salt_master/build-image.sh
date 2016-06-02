TAG=${DOCKER_IMAGE_TAG:-"latest"}
sudo docker build -t="gtoff/salt_master:${TAG}" .
