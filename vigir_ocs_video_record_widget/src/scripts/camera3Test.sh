#!/bin/bash
plugctl -n 1 oPCR[0].bcast_connection=0
dvgrab -format dv1 -guid 08004601043d97ca - | playdv --disable-audio --no-mmap -d 1
