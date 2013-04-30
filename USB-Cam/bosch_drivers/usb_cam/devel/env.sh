#!/usr/bin/env sh
# generated from catkin/cmake/templates/env.sh.in

if [ $# -eq 0 ] ; then
  /bin/echo "Entering environment at '/home/rsd/groovy_workspace/FroboMind-Fuerte/USB-Cam/bosch_drivers/usb_cam/devel', type 'exit' to leave"
  . "/home/rsd/groovy_workspace/FroboMind-Fuerte/USB-Cam/bosch_drivers/usb_cam/devel/setup.sh"
  "$SHELL" -i
  /bin/echo "Exiting environment at '/home/rsd/groovy_workspace/FroboMind-Fuerte/USB-Cam/bosch_drivers/usb_cam/devel'"
else
  . "/home/rsd/groovy_workspace/FroboMind-Fuerte/USB-Cam/bosch_drivers/usb_cam/devel/setup.sh"
  exec "$@"
fi
