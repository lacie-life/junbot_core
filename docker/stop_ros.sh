#!/bin/bash

docker commit $(docker ps -l -q) aet_junbot:latest



