#!/bin/bash

FAIL_COUNT_FILE="/tmp/gps_fail_count"
FAIL_COUNT=`cat $FAIL_COUNT_FILE 2>/dev/null`

LATLON=`timeout 5s gpspipe -w -n 10|grep -om1 "[-]\?[[:digit:]]\{1,3\}\.[[:digit:]]\{9\}"`
LAT=`echo -e "$LATLON" | sed -n -e 1p`
LON=`echo -e "$LATLON" | sed -n -e 2p`
if [ -z "$LAT" ]; then
  LAT=0
fi
if [ -z "$LON" ]; then
  LON=0
fi
if [ $LAT = 0 -o $LON = 0 ]; then
  echo "GPS is invalid"
  FAIL_COUNT=$(($FAIL_COUNT + 1))
else
  echo "GPS is valid"
  FAIL_COUNT=0
fi
echo $FAIL_COUNT > $FAIL_COUNT_FILE

if [ $FAIL_COUNT -gt 5 ]; then
  echo "reset GPS"
  gpio mode 21 down
  sleep 1
  gpio mode 21 up
  echo 0 > $FAIL_COUNT_FILE
fi