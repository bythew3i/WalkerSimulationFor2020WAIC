
# TheWalkingDead Docs

- [Resources](#resources)
- [Setup](#setup)
- [Run Walk Demo](#run-walk-demo)

## Resources

- [ROS_tutorial](https://www.youtube.com/watch?v=Q5y-3aZdzfQ&list=PLJNGprAk4DF5PY0kB866fEZfz6zMLJTF8)
- [rospy publisher/subscriber](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers)
- [rospy services](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)


## Setup
**If `roscore` is NOT runing**, run this in another terminal:
```bash
roscore
```

Before using `thewalkingdead` package, you need to compile source once:
```bash
cd { PATH }/ubt_sim_ws

# make under /ubt_sim_ws
catkin_make
source devel/setup.bash
```

Then you can change directory to `thewalkingdead` package:
```bash
roscd thewalkingdead
```



## Run Walk Demo
Auth to use leg_motion
```bash
roslaunch leg_motion walker2_leg.launch account_file:={ Path }/user_account.json
```

Run python script
```bash
roscd thewalkingdead
rosrun thewalkingdead walk_demo.py
```


