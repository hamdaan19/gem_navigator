docker run -it -dt --name gem_nav_container \
  --env="DISPLAY" --net host \
  -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/dri:/dev/dri \
  -v /home/hamdaan/ROS/gemdocker_ws:/home/gem_ws \
  gem_navigator_image /bin/zsh