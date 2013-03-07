#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/rsd/groovy_workspace/FroboMind-Fuerte/fmActuators/devel', type 'exit' to leave"
  . "/home/rsd/groovy_workspace/FroboMind-Fuerte/fmActuators/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/rsd/groovy_workspace/FroboMind-Fuerte/fmActuators/devel'"
else
  . "/home/rsd/groovy_workspace/FroboMind-Fuerte/fmActuators/devel/setup.sh"
  exec "$@"
fi
