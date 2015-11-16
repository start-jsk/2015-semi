20151104\_semi
==============

PointCloud2 simulator
------------------------
```sh
roslaunch baxter_vision_examples baxter_world.launch
roslaunch baxter_vision_examples heightmap.launch
```

rosbag launch
------------------------
```sh
cd /home/baxter/20151104_semi
roslaunch ./play.launch  # play log file
roslaunch ./setup.launch # setup recognition nodes
```


Download the rosbag file
------------------------

```sh
sudo pip install gdown
gdown "https://drive.google.com/uc?id=0B9P1L--7Wd2vaVhLa0pJUzJCUlk&export=download" -O 2015-11-04-19-37-29.bag
```
