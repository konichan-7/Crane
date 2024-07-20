docker run \
    --user=rm \
    --network=host \
    -it \
    -v $PWD:/sp_vision \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --env=DISPLAY \
    -w /sp_vision \
    -v /dev:/dev \
    --device-cgroup-rule='c *:* rmw' \
    --ipc host \
    --name auto_aim \
    --rm \
    sp_vision 