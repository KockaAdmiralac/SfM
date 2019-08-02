#!/bin/sh
rm -rf CMakeFiles
rm CMakeCache.txt
rm SfM
rm Makefile
cmake .
make
