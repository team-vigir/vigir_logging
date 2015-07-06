#!/bin/bash
VIDEO_SIZE="1280x960"
FPS="15"

STREAM_KEY=""
STREAM_URL="/$STREAM_KEY"
avconv -f x11grab -s "$VIDEO_SIZE" -r "$FPS" -i :0.0 -vcodec libx264 -f flv "$STREAM_URL"
