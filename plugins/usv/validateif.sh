#!/bin/bash

if [ ! -e /dev/modem ]; then
  echo "no modem"
  exit 0
fi

IF_FAIL_COUNT_FILE="/tmp/if_fail_count"
IF_FAIL_COUNT=`cat $IF_FAIL_COUNT_FILE 2>/dev/null`

if ping -q -c 1 -W 1 8.8.8.8 > /dev/null; then
  echo "IPv4 is up"
  echo 0 > $IF_FAIL_COUNT_FILE
else
  echo "IPv4 is down"
  echo $(($IF_FAIL_COUNT + 1)) > $IF_FAIL_COUNT_FILE
fi

if [ $IF_FAIL_COUNT -gt 5 ]; then
  echo "reset usb hub"
  gpio mode 22 down
  sleep 1
  gpio mode 22 up
  echo 0 > $IF_FAIL_COUNT_FILE
fi