
# TheWalkingDead Docs

- [Resources](#resources)
- [Setup](#setup)
- [Run Walk Demo](#run-walk-demo)
- [Run Task 1](#run-task-1)

## Resources

- [ROS_tutorial](https://www.youtube.com/watch?v=Q5y-3aZdzfQ&list=PLJNGprAk4DF5PY0kB866fEZfz6zMLJTF8)
- [Writing a Simple Publisher and Subscriber (Python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
- [Writing a Simple Service and Client (Python)](http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28python%29)


## Setup
**If `roscore` is NOT runing**, run this in another terminal:
```bash
roscore
```

Before using `thewalkingdead` package, you need to compile source once:
```bash
cd { PATH }/ubt_sim_ws

# catkin_make under /ubt_sim_ws
catkin_make
source devel/setup.bash
```



## Run Walk Demo
```bash
# Auth to use leg_motion
roslaunch leg_motion walker2_leg.launch account_file:={ Path }/user_account.json

# Run python script
rosrun thewalkingdead walk_demo.py
```

## Use of Inverse Kinematics
```bash
# Build service, in ${ubt_sim_ws}
catkin_make

# Run service
source ${ubt_sim_ws_HOME}/devel/setup.bash
roslaunch thewalkingdead solver_server_node.launch urdf_path:=${path_to_walker.urdf}
# [ INFO] [1592677570.580549330]: Ready to Solve.

# IK Service name is "inverse_kinematic_solver"
# FK Service name is "forward_kinematic_solver"
# Message type is "Solver", read more in thewalkingdead/srv/Solver.srv
#   Note that both IK and FK share the same service, see ik_demo.py for 
#   usage of both.
```

## Run Task 1
```bash
roslaunch thewalkingdead task1.launch
```

## Run Task 2
```bash
roslaunch thewalkingdead task2.launch
```

## Run Task 3
> TODO

## Run Task 4
> TODO

## Run Task 5
> TODO

## Run Task 6
```bash
roslaunch thewalkingdead task6.launch
```

## Run Task 7
> TODO

## Run Task 8
> TODO

## Run Task 8
> TODO

## Run Task 10
> **Attention:**
> You may notice the program is **not** excuted right away. Please allow some time to start all the required services (IK/FK solver and legmotion). Once after 10 seconds waiting, the task 10 will start automaticly.

```bash
# Wait first 10 seconds to start the required services
roslaunch thewalkingdead task10.launch
```

优化：
- 最后几步可以增加 阻尼系数，从而快速拉开完成比赛，缩小用时

## Run Task 11
```bash
roslaunch thewalkingdead task11.launch
```

## Run Task 12
> TODO

## Run Task 13
> TODO

## Run Task 14
```bash
roslaunch thewalkingdead task14.launch
```
优化:
- 可以试着增加阻抗提高重心稳定度，然后可以增大脚步，以减少比赛用时

Python Dependancy:
- from scipy.interpolate import KroghInterpolator

## Run Task 15
```bash
roslaunch thewalkingdead task15.launch
```

Notes:
- 6秒左右完成任务
- 没有做优化， 具有不稳定性，有一定可能会摔倒
- 采用倒立摆模型
- 除了Ref中的dynamic时候的步态规划， 还分别增加了起步与止步的步态规划
- 参考: 
    - 双足机器人爬楼梯步态规划与参数 吴飞 （文中起码有超出10处错误）
    - 双足机器人不行仿真预实验研究 史耀强

Python Dependancy:
- from scipy.interpolate import KroghInterpolator