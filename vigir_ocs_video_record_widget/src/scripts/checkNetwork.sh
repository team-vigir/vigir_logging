foo=$(nmcli con status id $1)
if [ ! ${#foo} -gt 0 ]
then 
	echo "setting up $1 network.."
	nmcli con up id $1
else
	echo "Already on $1 network."
fi
echo "Done!"
