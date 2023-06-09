# multiagent-automation
run multiple autoware agents in CARLA simulator

## setup
Follow the CARLA-Autoware setup instructions [here](https://github.com/carla-simulator/carla-autoware)

## Shell script modification
In the original run.sh, add a bash command which specifies the ROS master in the docker image and starts the agent, as shown in run_hero.sh and run_hero1.sh\
Please note we need different agent names in different docker images

# Run multiple agents

1. Run a CARLA server.

```
./CarlaUE4.sh
```

2. Run multiple `carla-autoware` images and in each of them run an Autoware agent using different ROS masters: 

```sh
./run_test.sh
```
This opens two tabs, one for run_hero.sh and one for run_hero1.sh\
I'm using mate terminal here, please modify accordingly.


