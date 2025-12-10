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

echo_and_run cd "/home/ROS/workspace/src/ROS-TCP-Endpoint"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ROS/workspace/install_isolated/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ROS/workspace/install_isolated/lib/python3/dist-packages:/home/ROS/workspace/build_isolated/ros_tcp_endpoint/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ROS/workspace/build_isolated/ros_tcp_endpoint" \
    "/usr/bin/python3" \
    "/home/ROS/workspace/src/ROS-TCP-Endpoint/setup.py" \
    egg_info --egg-base /home/ROS/workspace/build_isolated/ros_tcp_endpoint \
    build --build-base "/home/ROS/workspace/build_isolated/ros_tcp_endpoint" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ROS/workspace/install_isolated" --install-scripts="/home/ROS/workspace/install_isolated/bin"
