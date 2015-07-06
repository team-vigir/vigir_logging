#!/bin/bash
cd $1
quality=5000k
if [ ! -d $1/archive  -a ! $2 == "true" ]; then
    mkdir archive
fi
for file in $1*.avi ;
do
	parts=($file//_/ })
	if [ $# -eq 3 ];
then

avconv -i $file -vf drawtext="fontcolor=white: fontsize=16: fontfile=/usr/share/fonts/truetype/ttf-dejavu/DejaViSans.ttf: box=1:boxcolor=black@0.55:x=50:y=20:timecode='00\\:02\\:10\\:02':rate=29.9" -y $file.mkv       
fi
if [ ! $2 == "true" ]; then
            mv $file $1/archive
        else
            rm $file
        fi
done
