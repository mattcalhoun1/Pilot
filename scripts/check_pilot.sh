#!/bin/bash

pidcount=$(ps -ef | grep drive_cm4.py | grep -v grep | wc -l)
echo $pidcount
if [ "$pidcount" -gt "0" ]; then 
  echo "Pilot is running"
else
  echo "Pilot is not running, starting it"
  cd /home/matt/projects/Pilot
  ./start_pilot_cm4.sh & 
fi
