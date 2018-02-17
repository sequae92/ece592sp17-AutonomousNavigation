#!/bin/bash
if [ $# = 0 ]
then
	echo "Usage ./swapMemory ON/OFF to add/remove swap memory."
else 
	if [ $1 = "ON" ]
	then
		sudo fallocate -l 8G /swapfile
		sudo chmod 600 /swapfile
		sudo mkswap /swapfile
		sudo swapon /swapfile
	else
		sudo swapoff /swapfile
		sudo rm -rf /swapfile
	fi
fi
