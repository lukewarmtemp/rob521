# Set Up Dev Env

## Ubuntu

### 1. Install Docker

```
$ curl -fsSL https://get.docker.com -o get-docker.sh
$ sudo sh ./get-docker.sh
```

### 2. Add users to group
```
$ sudo groupadd docker
$ sudo usermod -aG docker $USER
```

# Docker Image

## Building Image

```
docker build -t rob_dev .
```

## Tag Image

```
docker tag rob_dev:latest lukewarmtemp/rob521:lab01
```

# Binding Directorydocker build -t rob_dev .

```
$ docker run -it --user ros --network=host --ipc=host \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY \
-e LIBGL_ALWAYS_SOFTWARE=1 \
-v ./catkin_ws:/catkin_ws lukewarmtemp/rob521:lab01
```
Note: you can add optional argumenents such as `roscore` to execute the command in the container, and then immediately exit.