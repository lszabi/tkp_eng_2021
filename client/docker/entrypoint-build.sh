#!/bin/bash
set -e

TORCS_CLIENT_DIR=torcs_client

echo ""
echo "----------------------------"
echo "Move to torcs_client folder"
echo "----------------------------"
cd $TORCS_CLIENT_DIR
pwd

echo ""
echo "----------------------------"
echo "Build client in docker container"
echo "----------------------------"
make -f Makefile.linux 2>&1 | tee make.docker.linux.log
echo "----------------------------"

echo ""
echo "===> build DONE"
echo ""
