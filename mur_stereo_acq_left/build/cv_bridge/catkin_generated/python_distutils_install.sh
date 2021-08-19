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

echo_and_run cd "/workspace/both_cam_acq/mur_stereo_acq_left/src/mur_stereo_basler/includes/cv_bridge"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/workspace/both_cam_acq/mur_stereo_acq_left/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/workspace/both_cam_acq/mur_stereo_acq_left/install/lib/python2.7/dist-packages:/workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge" \
    "/usr/bin/python2" \
    "/workspace/both_cam_acq/mur_stereo_acq_left/src/mur_stereo_basler/includes/cv_bridge/setup.py" \
     \
    build --build-base "/workspace/both_cam_acq/mur_stereo_acq_left/build/cv_bridge" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/workspace/both_cam_acq/mur_stereo_acq_left/install" --install-scripts="/workspace/both_cam_acq/mur_stereo_acq_left/install/bin"
