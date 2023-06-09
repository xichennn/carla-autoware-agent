# multiagent-automation
run multiple autoware agents in CARLA simulator

## setup
Follow the CARLA-Autoware setup instructions [here](https://github.com/carla-simulator/carla-autoware)

## Shell script modification
In the original run.sh, 

# Run multiple agents

1. Run a CARLA server.

```
./CarlaUE4.sh
```

2. Run the `carla-autoware` image using ROS master : 

```sh
./run_hero.sh
```

This will start an interactive shell inside the container. To start the agent run the following command:

```sh
roslaunch carla_autoware_agent carla_autoware_agent.launch town:=Town01
```
Bash script run_hero initiate 
