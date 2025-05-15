### F1Tenth - Roboracer - Lab1

Python implementation of the lab 1 assignment 

Takeaways
1. talker_node: talker.py -- {publishes value to "/drive"}
2. relay_node:  relay.py -- {subscribes to "/drive"; multiplies with 3; publishes to "/drive_relay"}
3. launch_file: lab1_launch.py

```
ros2 run lab1_pkg talker
ros2 run lab1_pkg relay
```

Launch file
```
ros2 launch lab1_pkg lab1_launch.py
```
