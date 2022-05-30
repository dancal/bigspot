#!/bin/bash

for i in {0..60}
do
    PS4=`/usr/bin/bluetoothctl paired-devices | /usr/bin/grep "Wireless Controller" | wc -l`
    echo $PS4
    if [ $PS4 -ge 1 ]
    then
        exit
    fi

sudo /usr/bin/bluetoothctl <<EOF
power on
discoverable on
pairable on
connect A0:5A:5C:D2:3C:73
trust A0:5A:5C:D2:3C:73
EOF

    sleep 2

done

