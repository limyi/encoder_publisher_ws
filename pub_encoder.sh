#!/bin/bash
IDs=()
indicator=1
<<COMMENT
while [ $indicator -ne 0 ]
do
    echo "Enter Encoder ID, enter 0 when done: "
    read id
    if [ $id -eq 0 ]
        then let indicator=0
    else
        IDs+=( $id )
    fi
done
echo ${IDs[*]}

for id in ${IDs[@]};
do
    cansend can0 $id#2300180510
    cansend can0 000#01
done

cansend can0 601#2300180510
cansend can0 602#2300180510
cansend can0 603#2300180510
cansend can0 605#2300180510
COMMENT
cansend can0 000#0100
sleep 2
cansend can0 000#0100
sudo chmod 666 /dev/ttyUSB*
