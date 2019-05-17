#!/bin/bash

if [ ! -e /dev/modem ]; then
  echo "no modem"
  exit 0
fi

FAIL_COUNT_FILE="/tmp/network_fail_count"
FAIL_COUNT=`cat $FAIL_COUNT_FILE 2>/dev/null`

if ping -q -c 1 -W 1 8.8.8.8 > /dev/null; then
  echo "NETWORK is valid"
  FAIL_COUNT=0
else
  echo "NETWORK is invalid"
  FAIL_COUNT=$(($FAIL_COUNT + 1))
fi
echo $FAIL_COUNT > $FAIL_COUNT_FILE

if [ $FAIL_COUNT -gt 5 ]; then
  echo "reset USB"
  gpio mode 22 down
  sleep 1
  gpio mode 22 up
  echo 0 > $FAIL_COUNT_FILE
fi