#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/sutd/encoder_publisher_ws/src/roboclaw_node"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/sutd/encoder_publisher_ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/sutd/encoder_publisher_ws/install/lib/python2.7/dist-packages:/home/sutd/encoder_publisher_ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/sutd/encoder_publisher_ws/build" \
    "/usr/bin/python2" \
    "/home/sutd/encoder_publisher_ws/src/roboclaw_node/setup.py" \
     \
    build --build-base "/home/sutd/encoder_publisher_ws/build/roboclaw_node" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/sutd/encoder_publisher_ws/install" --install-scripts="/home/sutd/encoder_publisher_ws/install/bin"
