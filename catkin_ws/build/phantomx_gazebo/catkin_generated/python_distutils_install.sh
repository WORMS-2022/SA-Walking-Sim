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

echo_and_run cd "/home/worms/Desktop/Worms_2-legged/SA-Walking-Sim-d1eee7d6278811e94045d7ec845db312943924e8/catkin_ws/src/phantomx_gazebo"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/worms/Desktop/Worms_2-legged/SA-Walking-Sim-d1eee7d6278811e94045d7ec845db312943924e8/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/worms/Desktop/Worms_2-legged/SA-Walking-Sim-d1eee7d6278811e94045d7ec845db312943924e8/catkin_ws/install/lib/python3/dist-packages:/home/worms/Desktop/Worms_2-legged/SA-Walking-Sim-d1eee7d6278811e94045d7ec845db312943924e8/catkin_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/worms/Desktop/Worms_2-legged/SA-Walking-Sim-d1eee7d6278811e94045d7ec845db312943924e8/catkin_ws/build" \
    "/usr/bin/python3" \
    "/home/worms/Desktop/Worms_2-legged/SA-Walking-Sim-d1eee7d6278811e94045d7ec845db312943924e8/catkin_ws/src/phantomx_gazebo/setup.py" \
     \
    build --build-base "/home/worms/Desktop/Worms_2-legged/SA-Walking-Sim-d1eee7d6278811e94045d7ec845db312943924e8/catkin_ws/build/phantomx_gazebo" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/worms/Desktop/Worms_2-legged/SA-Walking-Sim-d1eee7d6278811e94045d7ec845db312943924e8/catkin_ws/install" --install-scripts="/home/worms/Desktop/Worms_2-legged/SA-Walking-Sim-d1eee7d6278811e94045d7ec845db312943924e8/catkin_ws/install/bin"
