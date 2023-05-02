#!/bin/bash

docker build -t ros_serial_update .

docker run -d --name ros_serial_update ros_serial_update roscore

docker cp ros_serial_update:/tmp/ros_lib ./

docker stop ros_serial_update

docker rm ros_serial_update

cp -Rf ./ros_lib/* ./src

cp -Rf ./ros_lib/examples/* ./examples

cp -Rf ./ros_lib/tests/* ./extras/tests

rm -Rf ./src/examples

rm -Rf ./src/tests

rm -Rf ./ros_lib