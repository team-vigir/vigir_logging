#!/bin/bash
plugctl -n 0 oPCR[2].bcast_connection=0
dvgrab -format dv1 -guid 08004601043d97d7 - | playdv --disable-audio --no-mmap -d 1
