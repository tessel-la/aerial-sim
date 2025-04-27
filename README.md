# Aerostack2 Simulator

This repository provides the simulation environment specifically configured and tested for the [Robo-Boy](c:\Users\ricca\Documents\GitHub\robo-boy) project. It utilizes components and setup based on the Aerostack framework.

### Setup using Docker 

All the demo is designed to be easily used and modify using docker. Go to the root folder of the repository and run:

```bash
xhost + # this will enable gazebo visualization
docker compose up -d # use the -d for keep the container alive in background
```

> If your host machine is Windows, change display configuration at `docker-compose.yaml`. See [Issue 4](https://github.com/aerostack2/demo_ROSConES24/issues/4). Kudos to [@dvdmc](https://github.com/dvdmc).

With this there is a running instance of the container with this project mounted in ```/root/demo_ROSConES24```.
Now you can run as much terminals as you need by running: 

```bash
docker exec -it aerostack2_sim /bin/bash
```

Once inside the container 

```bash
cd simulation
tmuxinator start -p aerostack2.yml
```