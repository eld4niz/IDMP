
if [ -z "$SUDO_USER" ]
then
      user=$USER
else
      user=$SUDO_USER
fi

xhost +local:root
XAUTH=/tmp/.docker.xauth

if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    sudo chmod a+r $XAUTH
fi

sudo docker run -it --rm \
        --name=idmp \
        --shm-size=1g \
        --ulimit memlock=-1 \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --privileged \
        --volume="/home/$user/idmp_shared:/root/Shared:rw" \
        --device=/dev/usb \
        --device=/dev/video0 \
        --gpus all \
        --env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --env="NVIDIA_VISIBLE_DEVICES=all" \
        --env="NVIDIA_DRIVER_CAPABILITIES=all" \
        --network=host \
        --ipc=host --privileged \
        idmp_ros \
        bash
