#!/bin/bash

numLine=1
cat $1 | while read line
do
    echo $numLine: $line # 出力
    numLine=$((numLine + 1)) # 行数を1増やす
    echo $line > $2
    sleep 0.1
done
