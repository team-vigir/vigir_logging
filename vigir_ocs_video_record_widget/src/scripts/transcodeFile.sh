#!/bin/bash
cd $1
if [ ! -d $1/archive  -a ! $2 == "true" ]; then
    mkdir archive
fi
for file in $1*.avi ;
do
	avconv -i $file -y $file.mkv
        if [ ! $2 == "true" ]; then
            mv $file $1/archive
        else
            rm $file
        fi
done
