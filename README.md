# Cartographer with Auto-Navigation

&emsp;This package is an extend Cartographer_ros package for robot auto-patrol. 
In this package, we add a voice command for robot. Otherwise, it should cooperate
with follow ROS packages. Back to talk about this package, we finished a **Nav goal labelling**
, **Auto patrol**,**Mapping**,**Localizaton initial pose** and **Voice command navigation**. We tested those functions on AutoLabor Robot Platform and ROS Kinetic.

---------
## Dependency
- [Cartographer](https://github.com/googlecartographer/cartographer)
- [baidu_speech](https://github.com/Miaowaaaa/baidu_speech)
- *[voice_command](https://github.com/Miaowaaaa/voice_command_ros). You can also choose this one to replace `baidu_speech`.
- [rviz_navigation_plugin](https://github.com/Miaowaaaa/rviz_navigation_plugin)
- A Real Robot Driver
-----
## How to use
&emsp;For starting auto-patrol, make sure you have compiled front dependencies correctly.

```
# build a map
1. roslaunch cartographer_ros autolabor_cartographer.launch
# save map
2. rosservice call /finish_trajectory "stem:'good'"
# switch to package dir
3. cd ../cartographer_ros
# move map to maps folder
4. mv ~/.ros/good.pgm ../cartographer_ros/maps/good.pgm 
5. mv ~/.ros/good.yaml ../cartographer_ros/maps/good.yaml
# start patrol
6. roslaunch cartographer_ros cartographer_navigaiton.launch
```
------
## Subscribe

### set_points
- /amcl_pose   
use to record `2D Estimate Pose` on `rviz` to initial robot pose.

- /my_nav_goal  
use to record navigation goals published by `rviz_navigation_plugin`.

- /point_label  
use to record labels of navigation goals published by `rviz_navigation_plugin`.

- /clear_nav_point  
listen to message of dropping all goals for patrol.

### auto_patrol
- /reg_result  
listen to the result of voice recogniton.
----
## Publish
### auto_patrol
- /initialpose  
to initialize the robot pose with the last record.
----
## Service
### auto_patrol
- patrol_service
the service for robot auto-patrol
----

##More
There maybe bugs need to fixed.
