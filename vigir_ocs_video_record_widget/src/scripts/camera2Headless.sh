#!/bin/bash
cd $1
plugctl -n 0 oPCR[0].bcast_connection=0
dvgrab -format dv1 -guid 08004601043d97ca - | dvgrab -format dv1 -stdin $2_
