#!/bin/sh
set -e
cd `dirname $0`

METHOD=$1
RANSAC=$2
RANSAC_1=$3
RANSAC_2=$4
RANSAC_3=$5
MATCHER=$6
THRESHOLD=$7
SEQUENCE=$8

# sift-our-8-100-23-thres7-seq04
FOLDER="../metrics/$METHOD-$RANSAC-$RANSAC_1-$RANSAC_2-$RANSAC_3-$MATCHER-thres$THRESHOLD-seq$SEQUENCE"
printf -v SEQUENCE_FORMAT "%02d" "$SEQUENCE"
mkdir -p $FOLDER
mkdir -p "TEMP/TRIANGULATION/$SEQUENCE_FORMAT"
./clean.sh
cmake -DCMAKE_BUILD_TYPE=Debug -DSEQUENCE=$SEQUENCE -DRATIO_THRESH="${THRESHOLD}f" -DRANSAC_1=$RANSAC_1 -DRANSAC_2=$RANSAC_2 -DRANSAC_3=$RANSAC_3 -DMETHOD=$METHOD -DMATCHER=$MATCHER -DRANSAC=$RANSAC .
make -j4
./SfM
./ICP > "$FOLDER/icp.txt"
cp "TEMP/MATRICES/$SEQUENCE_FORMAT.txt" "$FOLDER/matrix.txt"
cp "TEMP/PERFORMANCE/$SEQUENCE_FORMAT.txt" "$FOLDER/perf.txt"
cp "TEMP/METRICS/$SEQUENCE_FORMAT.txt" "$FOLDER/metrics.txt"
python3 merge.py $SEQUENCE
mv merge.ply "$FOLDER/cloud.ply"
python3 metrics.py $SEQUENCE >> "$FOLDER/metrics.txt"
mv rot-rpe.svg $FOLDER
mv trans-rpe.svg $FOLDER
cp -r "TEMP/TRIANGULATION/$SEQUENCE_FORMAT" "$FOLDER/clouds"
