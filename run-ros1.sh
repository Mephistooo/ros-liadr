# where the project resides inside docker
DOCKER_ROOT="/workspace/src/ros_bot"

# generate mount commands
DATA_VOLUME="--volume $PWD/data:$DOCKER_ROOT/data"
DEV_VOLUME=""

# check for V4L2 devices
V4L2_DEVICES=" "

for i in {0..9}
do
	if [ -a "/dev/video$i" ]; then
		V4L2_DEVICES="$V4L2_DEVICES --device /dev/video$i "
	fi
done

echo "V4L2_DEVICES:  $V4L2_DEVICES"

# give docker root user X11 permissions
sudo xhost +si:localuser:root

# enable SSH X11 forwarding inside container (https://stackoverflow.com/q/48235040)
XAUTH=/tmp/.docker.xauth
sudo rm -rf $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod 777 $XAUTH


MOUNTS="\
--device /dev/ttyUSB0 \
--device /dev/snd \
--device /dev/bus/usb \
--volume /etc/timezone:/etc/timezone:ro \
--volume /etc/localtime:/etc/localtime:ro \
$DEV_VOLUME \
$DATA_VOLUME \
$USER_VOLUME \
$V4L2_DEVICES"



sudo docker run --runtime nvidia -it --rm --name ros1-bridge-hector \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix/:/tmp/.X11-unix \
    -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v /etc/enctune.conf:/etc/enctune.conf \
    $MOUNTS ros1-bridge
