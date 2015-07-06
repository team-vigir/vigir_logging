#!/bin/sh
# Insert correct Camera Id for the coresponding cameras here.
cam1=$(plugreport | grep -o 0x08004601043d97d7)
cam3=$(plugreport | grep -o 0x08004601043d97d8)
cam2=$(plugreport | grep -o 0x08004601043d97ca)
cam4=$(plugreport | grep -o 0x08004601043d97d9)
cam1=${#cam1}
cam2=${#cam2}
cam3=${#cam3}
cam4=${#cam4}
output=0
if [ $cam1 -gt 0 ]
then
	echo "Camera 1 found!!"
	output=1
fi
if [ ${cam2} != 0 ]
then
	echo "Camera 2 Found!!"
	output=$(($output + 2))
fi
if [ ${cam3} != 0 ]
then 
	echo "Camera 3 Found!!"
	output=$(($output + 4))
fi
if [ ${cam4} != 0 ]
then 
	echo "Camera 4 Found!!"
	output=$(($output + 8))
fi
echo returning $output
exit $output
