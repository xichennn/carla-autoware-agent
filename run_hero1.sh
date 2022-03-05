if [[ $DOCKER_VERSION < "19.03" ]] && ! type nvidia-docker; then
    RUNTIME="--gpus all"
else
    RUNTIME="--runtime=nvidia"
fi

xhost +local:docker

docker run \
    -it --rm \
    --volume=$(pwd)/autoware-contents:/home/autoware/autoware-contents:z \
    --volume=$(pwd)/shared_dir:/home/autoware/shared_dir:z \
    --env="DISPLAY=${DISPLAY}" \
    --privileged \
    --net=host \
    --name="hero1" \
    $RUNTIME \
    carla-autoware:latest\
    bash -it -c "export ROS_MASTER_URI=http://localhost:11310/; roslaunch carla_autoware_agent carla_autoware_agent.launch use_ground_truth_detection:=True use_ground_truth_localization:=True role_name:='hero1'"



