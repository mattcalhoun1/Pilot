#!/bin/bash

pidcount=$(ps -ef | grep manage.py | grep -v grep | wc -l)
echo $pidcount
if [ "$pidcount" -gt "0" ]; then 
  echo "Django is running"
else
  echo "Django is not running, starting it"
  cd /home/matt/projects/services/NavService/navsvc
  ./start_django.sh & 
fi
