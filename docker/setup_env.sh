#!/bin/bash

cd docker

docker buildx build \
    --load \
    --platform $(uname)/$(uname -m) \
    --build-arg UBUNTU_VERSION=20.04 \
    --build-arg CUDA_VERSION=11.8 \
    --build-arg CUDNN_VERSION=8.6.0 \
    --build-arg TENSORRT_VERSION=8.5.3 \
    --build-arg INSTALL_AS_DEV=1 \
    --tag lifezero/junbot-base \
    .

cd ..

echo "Done building docker image"