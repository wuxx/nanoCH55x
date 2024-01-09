#!/bin/bash

succ_count=0
fail_count=0

trap 'onCtrlC' INT
function onCtrlC () {
    echo 'exit'
    exit 0
}

while [ 1 ]; do
    ./flash_write_552.sh
    if [ $? -eq 0 ]; then

        succ_count=$(($succ_count+1))  
        echo -e "\033[32m---------- SUCC [$succ_count] ----------\033[0m"
        buzzer_succ

        #fail_count=$(($fail_count+1))  
        #echo -e "\033[31m---------- FAIL [$fail_count] ----------\033[0m"
        #buzzer_fail

    else
        echo "wait attach..."
        sleep 0.2

    fi

done
