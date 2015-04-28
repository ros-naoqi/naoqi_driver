touch .catkin
source setup.bash
if [ 0 -eq $# ]; then
  exec bin/alrosbridge_bin
else
  exec bin/alrosbridge_bin --qi-url=tcp://$1:9559
fi
