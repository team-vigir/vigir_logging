#!/bin/bash
cd $1
plugctl -n 0 oPCR[1].bcast_connection=0
dvgrab -format dv1 -guid 08004601043d97d7 - | dvgrab -format dv1 -stdin $2_
