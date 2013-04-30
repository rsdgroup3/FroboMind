#!/bin/sh -x

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

cd "/home/rsd/groovy_workspace/FroboMind-Fuerte/vision_opencv/image_geometry"

# todo --install-layout=deb per platform
# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/usr/local/lib/python2.7/dist-packages:/home/rsd/groovy_workspace/FroboMind-Fuerte/vision_opencv/image_geometry/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/rsd/groovy_workspace/FroboMind-Fuerte/vision_opencv/image_geometry" \
    "/usr/bin/python" \
    "/home/rsd/groovy_workspace/FroboMind-Fuerte/vision_opencv/image_geometry/setup.py" \
    build --build-base "/home/rsd/groovy_workspace/FroboMind-Fuerte/vision_opencv/image_geometry" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix=/usr/local --install-scripts=/usr/local/bin
