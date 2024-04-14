XAUTH=/tmp/.docker.xauth
if not exist %XAUTH% (
    set xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if not "%xauth_list%"=="" (
        echo %xauth_list% | xauth -f %XAUTH% nmerge -
    ) else (
        type nul > %XAUTH%
    )
)

docker stop turtlebot_mpcc_test || true
docker rm turtlebot_mpcc_test || true

docker run -it `
    --workdir="/workspace" `
    --env="DISPLAY=host.docker.internal:0" `
    --env="QT_X11_NO_MITSHM=1" `
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" `
    --env="XAUTHORITY=%XAUTH%" `
    --volume="%XAUTH%:%XAUTH%" `
    --privileged `
    --network=host `
    --name="turtlebot_mpcc_test" `
    ros2_mpcc_turtlebot `
    /bin/bash