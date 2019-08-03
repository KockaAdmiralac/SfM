#!/bin/sh
set -e
cd `dirname $0`
./clean.sh
cmake -DCMAKE_BUILD_TYPE=Debug .
make
