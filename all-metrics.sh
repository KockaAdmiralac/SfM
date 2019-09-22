#!/bin/sh
set -e
cd `dirname $0`

./metrics.sh orb opencv 8 100 23 bruteforce 0.8 4
./metrics.sh orb our 8 100 23 bruteforce 0.8 4
./metrics.sh sift opencv 8 100 23 bruteforce 0.7 4
./metrics.sh sift opencv 8 100 23 flann 0.7 4
./metrics.sh sift our 8 100 23 bruteforce 0.7 4
./metrics.sh sift our 8 100 23 flann 0.7 4
./metrics.sh surf opencv 8 100 23 bruteforce 0.7 4
./metrics.sh surf opencv 8 100 23 flann 0.7 4
./metrics.sh surf our 8 100 23 bruteforce 0.7 4
./metrics.sh surf our 8 100 23 flann 0.7 4
