#!/bin/bash
cd $1
plugctl -n 0oPCR[2].bcast_connection=0
dvgrab -format dv1 -guid 08004601043d97d7 - > dvgrab -format dv1 -stdin $2_
