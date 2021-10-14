#!/bin/bash
set -e

export CFLAGS="-fPIC"
export CPPFLAGS=$CFLAGS
export CXXFLAGS=$CFLAGS


echo ""
echo "----------------------------"
echo "Move to torcs folder"
echo "----------------------------"
cd torcs
pwd

echo ""
echo "----------------------------"
echo "Execuit Configuration"
echo "----------------------------"
./configure --prefix=$(pwd)/BUILD --exec_prefix=$(pwd)/BUILD /2>&1 | tee __configure.log

echo ""
echo "----------------------------"
echo "make/install/datainstall"
echo "----------------------------"
make 2>&1 | tee __make.log
echo "----------------------------"
make install 2>&1 | tee __install.log
echo "----------------------------"
make datainstall 2>&1 | tee __datainstall.log

echo ""
echo "===> build DONE"
echo ""
