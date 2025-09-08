#!/bin/bash

help()
{
  # Display Help
  echo "Wait until dashboard server is accepting commands"
  echo
  echo "Syntax: `basename "$0"` [-i]"
  echo "options:"
  echo "    -i     IP address to reach the robot"
  echo "    -h             Print this Help."
  echo
}

IP_ADDRESS=192.168.56.101

while getopts ":hi:" option; do
  case $option in
    h) # display Help
      help
      exit;;
    i) # IP address
      IP_ADDRESS=${OPTARG}
      ;;
    \?) # invalid option
      echo "Error: Invalid option"
      help
      exit;;
  esac
done

netcat -z $IP_ADDRESS 29999
while [ $? -eq 1 ]
do
    echo "Dashboard server not accepting connections..."
    sleep 3
    netcat -z $IP_ADDRESS 29999
done
echo "Dashboard server connections are possible."
sleep 5
