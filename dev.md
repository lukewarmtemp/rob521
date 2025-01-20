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
$ sudo udermod -aG docker $USER
```

# Docker Image

## Building Image

```
$ docker build -t rob498_dev .
```

# Binding Directory

```
$ docker run -it --user ros --network=host --ipc=host \
-v /tmp/.X11-unix:/tmp/.X11-unix:rw --env=DISPLAY \
-e LIBGL_ALWAYS_SOFTWARE=1 \
-v ./source:/mysource rob498_dev
```
Note: you can add optional argumenents such as `roscore` to execute the command in the container, and then immediately exit.