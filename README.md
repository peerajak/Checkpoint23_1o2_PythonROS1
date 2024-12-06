# Checkpoint23_1o2_PythonROS1
## Using Rostest

Terminal 1

```
source ~/simulation_ws/devel/setup.bash
roslaunch tortoisebot_gazebo tortoisebot_playground.launch
```
Terminal 2

```
source ~/simulation_ws/devel/setup.bash
rostest tortoisebot_waypoints waypoints_test.test --reuse-master
```

if you want to test passes, file waypoints_test.py, set MAKE_TEST_PASS=True.

if you want to test fails, file waypoints_test.py, set MAKE_TEST_PASS=False.

## Manual Testing

Terminal 1

```
source ~/simulation_ws/devel/setup.bash
roslaunch tortoisebot_gazebo tortoisebot_playground.launch
```

Terminal 2

```
source ~/simulation_ws/devel/setup.bash
rosrun tortoisebot_waypoints tortoisebot_action_server.py
```

Terminal 3

Test with normal tortoisebot_action_client

```
source ~/simulation_ws/devel/setup.bash
rosrun tortoisebot_waypoints tortoisebot_action_client.py
```

Test with Unit test 

```
source ~/simulation_ws/devel/setup.bash
rosrun tortoisebot_waypoints waypoints_test.py
```





